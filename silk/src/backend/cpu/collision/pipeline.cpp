#include "backend/cpu/collision/pipeline.hpp"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <atomic>
#include <cassert>
#include <limits>
#include <vector>

#include "backend/cpu/collision/broadphase.hpp"
#include "backend/cpu/collision/collision.hpp"
#include "backend/cpu/collision/interval_root_finder.hpp"
#include "backend/cpu/collision/mesh_collider.hpp"
#include "backend/cpu/collision/narrowphase.hpp"
#include "backend/cpu/collision/object_collider.hpp"
#include "common/logger.hpp"

namespace silk::cpu {

/// @brief Filter function for object-level collision detection.
///
/// @param a First object collider
/// @param b Second object collider
/// @return true if collision should be tested, false to skip
bool object_collision_filter(const ObjectCollider& a, const ObjectCollider& b) {
  // Collision rules:
  // - Group = -1 means collision is disabled completely
  // - Objects must belong to the same collision group otherwise
  // - At least one object must be physical (not pure obstacle)
  return (a.group != -1 && b.group != -1 && a.group == b.group &&
          !(a.state_offset == -1 && b.state_offset == -1));
};

/// @brief Filter function for mesh-level inter-object collision.
///
/// @param a First mesh collider
/// @param b Second mesh collider
/// @return True if collision should be tested, false to skip
bool mesh_inter_collision_filter(const MeshCollider& a, const MeshCollider& b) {
  // Enforces three filtering rules:
  // 1. Only allows point-triangle and edge-edge collision types
  // 2. Rejects collisions where all vertices are pinned (zero inverse mass)

  if (a.type == MeshColliderType::Point &&
      b.type == MeshColliderType::Triangle) {
    bool is_both_pinned =
        (a.inv_mass(0) + b.inv_mass(0) + b.inv_mass(1) + b.inv_mass(2) == 0.0f);
    return (!is_both_pinned);
  }

  if (a.type == MeshColliderType::Triangle &&
      b.type == MeshColliderType::Point) {
    bool is_both_pinned =
        (a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2) + b.inv_mass(0) == 0.0f);
    return (!is_both_pinned);
  }

  if (a.type == MeshColliderType::Edge && b.type == MeshColliderType::Edge) {
    bool is_both_pinned =
        (a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) + b.inv_mass(1) == 0.0f);
    return (!is_both_pinned);
  }
  return false;
};

/// @brief Filter function for mesh self-collision detection.
///
/// @param a First mesh primitive
/// @param b Second mesh primitive
/// @return true if collision should be tested
bool mesh_self_collision_filter(const MeshCollider& a, const MeshCollider& b) {
  // Enforces three filtering rules:
  // 1. Only allows point-triangle and edge-edge collision types
  // 2. Rejects collisions between topologically adjacent primitives
  // 3. Rejects collisions where all vertices are pinned (zero inverse mass)

  if (a.type == MeshColliderType::Point &&
      b.type == MeshColliderType::Triangle) {
    bool is_neighbor = (a.index(0) == b.index(0) || a.index(0) == b.index(1) ||
                        a.index(0) == b.index(2));
    bool is_both_pinned =
        (a.inv_mass(0) + b.inv_mass(0) + b.inv_mass(1) + b.inv_mass(2) == 0.0f);
    return (!is_neighbor && !is_both_pinned);
  }

  if (a.type == MeshColliderType::Triangle &&
      b.type == MeshColliderType::Point) {
    bool is_neighbor = (b.index(0) == a.index(0) || b.index(0) == a.index(1) ||
                        b.index(0) == a.index(2));
    bool is_both_pinned =
        (a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2) + b.inv_mass(0) == 0.0f);
    return (!is_neighbor && !is_both_pinned);
  }

  if (a.type == MeshColliderType::Edge && b.type == MeshColliderType::Edge) {
    bool is_neighbor = (a.index(0) == b.index(0) || a.index(0) == b.index(1) ||
                        a.index(1) == b.index(0) || a.index(1) == b.index(1));
    bool is_both_pinned =
        (a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) + b.inv_mass(1) == 0.0f);
    return (!is_neighbor && !is_both_pinned);
  }

  return false;
};

/// Cull colliders using broadphase phase then call Function on each candidate.
template <typename Function>
void for_each_collision_candidate(Registry& registry,
                                  CollisionCache<MeshCollider>& mesh_ccache,
                                  Function&& function) {
  // Two phase broadphase: SAP object phase + KDTree mesh collider phase.

  auto object_colliders = registry.get_all_components<ObjectCollider>();
  CollisionCache<ObjectCollider> object_ccache;
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

  for (auto& ccache : object_ccache) {
    auto& oa = ccache.first;
    auto& ob = ccache.second;
    mesh_ccache.clear();
    KDTree<MeshCollider>::test_tree_collision(
        oa->mesh_collider_tree, ob->mesh_collider_tree,
        mesh_inter_collision_filter, mesh_ccache);

    int ccache_num = mesh_ccache.size();
    tbb::parallel_for(0, ccache_num, [&](int i) {
      auto& [ma, mb] = mesh_ccache[i];
      function(*oa, *ma, *ob, *mb);
    });
  }

  for (auto& o : object_colliders) {
    if (o.state_offset == -1 || !o.is_self_collision_on) {
      continue;
    }

    mesh_ccache.clear();
    o.mesh_collider_tree.test_self_collision(mesh_self_collision_filter,
                                             mesh_ccache);

    int ccache_num = mesh_ccache.size();
    tbb::parallel_for(0, ccache_num, [&](int i) {
      auto& [ma, mb] = mesh_ccache[i];
      function(o, *ma, o, *mb);
    });
  }
}

float CollisionPipeline::find_earliest_toi(Registry& registry,
                                           const Bbox& scene_bbox,
                                           float time_start, float max_time) {
  assert(time_start >= 0.0f && max_time >= 0.0f &&
         time_start + max_time <= 1.0f + ccd_tolerance);
  assert(ccd_minimum_separation_scale >= 0.0f);
  if (max_time == 0.0f) {
    return 0.0f;
  }

  // Get scene scale and min bbox padding
  Eigen::Vector3f abs_max =
      scene_bbox.min.cwiseAbs().cwiseMax(scene_bbox.max.cwiseAbs());
  float scene_bbox_padding = std::numeric_limits<float>::max();
  auto object_colliders = registry.get_all_components<ObjectCollider>();
  for (const auto& collider : object_colliders) {
    abs_max = abs_max.cwiseMax(
        collider.bbox.min.cwiseAbs().cwiseMax(collider.bbox.max.cwiseAbs()));
    scene_bbox_padding = std::min(scene_bbox_padding, collider.bbox_padding);
  }

  if (object_colliders.empty()) {
    return max_time;
  }

  Eigen::Array3f scene_ee_err = get_numerical_error(abs_max, false);
  Eigen::Array3f scene_vf_err = get_numerical_error(abs_max, true);
  float minimum_separation = ccd_minimum_separation_scale * scene_bbox_padding;

  std::atomic<float> earliest_toi{max_time};
  for_each_collision_candidate(
      registry, mesh_ccache_,
      [&](const ObjectCollider& oa, const MeshCollider& ma,
          const ObjectCollider& ob, const MeshCollider& mb) {
        float query_max_time = earliest_toi.load(std::memory_order_relaxed);
        auto toi = find_min_toi(oa, ma, ob, mb, time_start, query_max_time,
                                minimum_separation, ccd_tolerance, ccd_max_iter,
                                scene_ee_err, scene_vf_err);
        if (!toi) {
          return;
        }

        float previous = earliest_toi.load(std::memory_order_relaxed);
        while (*toi < previous &&
               !earliest_toi.compare_exchange_weak(previous, *toi,
                                                   std::memory_order_relaxed)) {
        }
      });
  return earliest_toi.load(std::memory_order_relaxed);
}

std::vector<Collision> CollisionPipeline::find_active_collisions(
    Registry& registry, float frame_time) {
  assert(frame_time >= 0.0f && frame_time <= 1.0f);
  assert(dcd_activation_distance_scale >= 0.0f);
  float scene_bbox_padding = std::numeric_limits<float>::max();
  auto object_colliders = registry.get_all_components<ObjectCollider>();
  for (const auto& collider : object_colliders) {
    scene_bbox_padding = std::min(scene_bbox_padding, collider.bbox_padding);
  }
  if (object_colliders.empty()) {
    return {};
  }
  float activation_distance =
      dcd_activation_distance_scale * scene_bbox_padding;

  tbb::enumerable_thread_specific<std::vector<Collision>> thread_collisions;
  for_each_collision_candidate(
      registry, mesh_ccache_,
      [&](const ObjectCollider& oa, const MeshCollider& ma,
          const ObjectCollider& ob, const MeshCollider& mb) {
        auto collision = find_active_collision(oa, ma, ob, mb, frame_time,
                                               activation_distance,
                                               collision_stiffness_base);
        if (collision) {
          thread_collisions.local().push_back(std::move(*collision));
        }
      });

  std::vector<Collision> collisions;
  for (auto& c : thread_collisions) {
    collisions.insert(collisions.end(), c.begin(), c.end());
  }
  return collisions;
}

}  // namespace silk::cpu
