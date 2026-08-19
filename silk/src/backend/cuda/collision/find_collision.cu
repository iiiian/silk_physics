#include "backend/cuda/collision/find_collision.cuh"

#include <Eigen/Core>
#include <cassert>
#include <vector>

#include "backend/cpu/collision/interval_root_finder.hpp"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/narrowphase.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/collision/oibvh.cuh"
#include "backend/cuda/collision/sap.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

__both__ bool object_collision_filter(const ObjectCollider& a,
                                      const ObjectCollider& b) {
  // Collision rules:
  // - Group = -1 means collision is disabled completely
  // - Objects must belong to the same collision group otherwise
  // - At least one object must be physical (not pure obstacle)
  return (a.group != -1 && b.group != -1 && a.group == b.group &&
          (a.is_physical || b.is_physical));
};

struct PTInterCollisionFilter {
  __both__ bool operator()(const TriangleCollider& a,
                           const PointCollider& b) const {
    bool is_both_pinned =
        ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
    return !is_both_pinned;
  }
};

struct PTSelfCollisionFilter {
  __both__ bool operator()(const TriangleCollider& a,
                           const PointCollider& b) const {
    bool is_both_pinned =
        ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
    bool is_neighbor = (a.index(0) == b.index || a.index(1) == b.index ||
                        a.index(2) == b.index);

    return !is_both_pinned && !is_neighbor;
  }
};

struct EEInterCollisionFilter {
  __both__ bool operator()(const EdgeCollider& a, const EdgeCollider& b) const {
    bool is_both_pinned = ((a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) +
                            b.inv_mass(1)) == 0.0f);
    return !is_both_pinned;
  }
};

struct EESelfCollisionFilter {
  __both__ bool operator()(const EdgeCollider& a, const EdgeCollider& b) const {
    bool is_both_pinned = ((a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) +
                            b.inv_mass(1)) == 0.0f);
    bool is_neighbor = (a.index(0) == b.index(0) || a.index(0) == b.index(1) ||
                        a.index(1) == b.index(0) || a.index(1) == b.index(1));
    return !is_both_pinned && !is_neighbor;
  }
};

/// Apply narrowphase operation PTFunction and EEFunction after broadphase
/// culling.
template <typename PTFunction, typename EEFunction>
void for_each_collision_candidate_batch(ObjRegistry& registry,
                                        int initial_cache_size,
                                        PTFunction&& pt_function,
                                        EEFunction&& ee_function,
                                        CudaRuntime rt) {
  auto object_colliders = registry.get_all_components<ObjectCollider>();

  SapCollisionCache<ObjectCollider> object_ccache;
  std::vector<int> object_proxies(object_colliders.size());
  for (int i = 0; i < object_colliders.size(); ++i) {
    object_proxies[i] = i;
  }
  int axis = sap_optimal_axis<ObjectCollider>(
      object_colliders, object_proxies.data(), object_proxies.size());
  sap_sort_proxies<ObjectCollider>(object_colliders, object_proxies.data(),
                                   object_proxies.size(), axis);
  sap_sorted_group_self_collision<ObjectCollider>(
      object_colliders, object_proxies.data(), object_proxies.size(), axis,
      object_collision_filter, object_ccache);

  auto pt_ccache = alloc<PTCCache>(rt, initial_cache_size);
  int pt_fill = 0;
  auto ee_ccache = alloc<EECCache>(rt, initial_cache_size);
  int ee_fill = 0;
  auto flush_pt = [&] {
    if (pt_fill == 0) {
      return;
    }
    pt_function(ctd::span(pt_ccache.data(), pt_fill));
    pt_fill = 0;
  };
  auto flush_ee = [&] {
    if (ee_fill == 0) {
      return;
    }
    ee_function(ctd::span(ee_ccache.data(), ee_fill));
    ee_fill = 0;
  };

  for (auto& [oa, ob] : object_ccache) {
    oa->triangle_collider_tree.test_ext_collision<PointCollider>(
        ob->point_colliders.value(), PTInterCollisionFilter{}, pt_ccache,
        pt_fill, rt);
    flush_pt();

    ob->triangle_collider_tree.test_ext_collision<PointCollider>(
        oa->point_colliders.value(), PTInterCollisionFilter{}, pt_ccache,
        pt_fill, rt);
    flush_pt();

    oa->edge_collider_tree.test_ext_collision<EdgeCollider>(
        ob->edge_collider_tree, EEInterCollisionFilter{}, ee_ccache, ee_fill,
        rt);
    flush_ee();
  }

  for (auto& object : object_colliders) {
    if (!object.is_physical || !object.is_self_collision_on) {
      continue;
    }
    object.triangle_collider_tree.test_ext_collision<PointCollider>(
        object.point_colliders.value(), PTSelfCollisionFilter{}, pt_ccache,
        pt_fill, rt);
    flush_pt();

    object.edge_collider_tree.test_self_collision(EESelfCollisionFilter{},
                                                  ee_ccache, ee_fill, rt);
    flush_ee();
  }
}

float find_min_toi(ObjRegistry& registry, int init_broadphase_cache_size,
                   float time_start, float max_time, float minimum_separation,
                   float ccd_tolerance, int ccd_max_iter, CudaRuntime rt) {
  assert(time_start >= 0.0f && max_time >= 0.0f &&
         time_start + max_time <= 1.0f + ccd_tolerance);
  assert(minimum_separation >= 0.0f);
  if (max_time == 0.0f) {
    return 0.0f;
  }

  auto object_colliders = registry.get_all_components<ObjectCollider>();
  if (object_colliders.empty()) {
    return max_time;
  }

  // Tight Inclusion uses a scene-wide magnitude to bound floating-point
  // evaluation error.
  Vec3f abs_max = Vec3f::zeros();
  for (const ObjectCollider& collider : object_colliders) {
    for (int axis = 0; axis < 3; ++axis) {
      float scene_min = collider.bbox.min(axis) + collider.bbox_padding;
      float scene_max = collider.bbox.max(axis) - collider.bbox_padding;
      abs_max(axis) = ctd::max(abs_max(axis), ctd::abs(scene_min));
      abs_max(axis) = ctd::max(abs_max(axis), ctd::abs(scene_max));
    }
  }
  Eigen::Vector3f eigen_abs_max{abs_max(0), abs_max(1), abs_max(2)};
  Vec3f ee_err = Vec3f::vec_like(
      cpu::get_numerical_error(eigen_abs_max, /*is_vertex_face=*/false));
  Vec3f vf_err = Vec3f::vec_like(
      cpu::get_numerical_error(eigen_abs_max, /*is_vertex_face=*/true));

  // Tight inclusion CCD is essentially DFS tree traversal, which is wayyy
  // faster on CPU. So we first cull cadidate on GPU via shallow BFS search then
  // resolve remaining queries on CPU.

  float min_toi = max_time;
  auto find_pt = [&](ctd::span<PTCCache> candidates) {
    min_toi =
        find_pt_min_toi(candidates, vf_err, time_start, max_time, min_toi,
                        minimum_separation, ccd_tolerance, ccd_max_iter, rt);
  };
  auto find_ee = [&](ctd::span<EECCache> candidates) {
    min_toi =
        find_ee_min_toi(candidates, ee_err, time_start, max_time, min_toi,
                        minimum_separation, ccd_tolerance, ccd_max_iter, rt);
  };
  for_each_collision_candidate_batch(registry, init_broadphase_cache_size,
                                     find_pt, find_ee, rt);
  return min_toi;
}

ctd::span<Collision> find_active_collisions(
    ObjRegistry& registry, int init_broadphase_cache_size, float time,
    float activation_distance, cu::device_buffer<Collision>& collision_storage,
    CudaRuntime rt) {
  assert(time >= 0.0f && time <= 1.0f);
  assert(activation_distance >= 0.0f);

  int collision_fill = 0;
  auto find_pt = [&](ctd::span<PTCCache> candidates) {
    find_pt_active_collisions(candidates, time, activation_distance,
                              collision_storage, collision_fill, rt);
  };
  auto find_ee = [&](ctd::span<EECCache> candidates) {
    find_ee_active_collisions(candidates, time, activation_distance,
                              collision_storage, collision_fill, rt);
  };
  for_each_collision_candidate_batch(registry, init_broadphase_cache_size,
                                     find_pt, find_ee, rt);
  return ctd::span<Collision>(collision_storage.data(), collision_fill);
}

}  // namespace silk::cuda
