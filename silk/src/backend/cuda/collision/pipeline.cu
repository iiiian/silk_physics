#include <cassert>

#include "backend/cpu/collision/broadphase.hpp"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/narrowphase.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/collision/pipeline.cuh"

namespace silk::cuda::collision {

__both__ bool object_collision_filter(const ObjectCollider& a,
                                      const ObjectCollider& b) {
  // Collision rules:
  // - Group = -1 means collision is disabled completely
  // - Objects must belong to the same collision group otherwise
  // - At least one object must be physical (not pure obstacle)
  return (a.group != -1 && b.group != -1 && a.group == b.group &&
          (a.is_physical || b.is_physical));
};

__both__ bool pt_inter_collision_filter(const TriangleCollider& a,
                                        const PointCollider& b) {
  bool is_both_pinned =
      ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
  return !is_both_pinned;
};

__both__ bool pt_self_collision_filter(const TriangleCollider& a,
                                       const PointCollider& b) {
  bool is_both_pinned =
      ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
  bool is_neighbor =
      (a.index(0) == b.index || a.index(1) == b.index || a.index(2) == b.index);

  return !is_both_pinned && !is_neighbor;
};

__both__ bool ee_inter_collision_filter(const EdgeCollider& a,
                                        const EdgeCollider& b) {
  bool is_both_pinned =
      ((a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) + b.inv_mass(1)) == 0.0f);
  return !is_both_pinned;
};

__both__ bool ee_self_collision_filter(const EdgeCollider& a,
                                       const EdgeCollider& b) {
  bool is_both_pinned =
      ((a.inv_mass(0) + a.inv_mass(1) + b.inv_mass(0) + b.inv_mass(1)) == 0.0f);
  bool is_neighbor = (a.index(0) == b.index(0) || a.index(0) == b.index(1) ||
                      a.index(1) == b.index(0) || a.index(1) == b.index(1));
  return !is_both_pinned && !is_neighbor;
};

int CollisionPipeline::find_collision(Registry& registry, float dt,
                                      cu::device_buffer<Collision>& collisions,
                                      CudaRuntime rt) {
  auto object_colliders = registry.get_all_components<ObjectCollider>();

  // Three-stage collision detection for inter-object collisions:
  // 1. Object-level broadphase using sweep-and-prune on CPU.
  // 2. Mesh-level broadphase using BVH-tree spatial queries on GPU.
  // 3. Narrowphase using continuous collision detection on GPU.

  // State 1. Object level broadphase on CPU.
  namespace cpu = ::silk::cpu;
  cpu::CollisionCache<ObjectCollider> object_ccache;
  std::vector<int> object_proxies(object_colliders.size());
  for (int i = 0; i < object_colliders.size(); ++i) {
    object_proxies[i] = i;
  }
  int axis = cpu::sap_optimal_axis<ObjectCollider>(
      object_colliders, object_proxies.data(), object_proxies.size());
  cpu::sap_sort_proxies<ObjectCollider>(object_colliders, object_proxies.data(),
                                        object_proxies.size(), axis);
  cpu::sap_sorted_group_self_collision<ObjectCollider>(
      object_colliders, object_proxies.data(), object_proxies.size(), axis,
      object_collision_filter, object_ccache);

  // Allocated device collision cache.
  auto pt_ccache =
      cu::make_buffer<ctd::pair<const TriangleCollider*, const PointCollider*>>(
          rt.stream, rt.mr, init_ccache_size, cu::no_init);
  int pt_ccache_fill = 0;
  auto ee_ccache =
      cu::make_buffer<ctd::pair<const EdgeCollider*, const EdgeCollider*>>(
          rt.stream, rt.mr, init_ccache_size, cu::no_init);
  int ee_ccache_fill = 0;
  int collision_fill = 0;

  // Stage 2-3. Gpu broadphase + narrowphase.

  for (auto& [oa, ob] : object_ccache) {
    oa->triangle_collider_tree
        .test_ext_collision<PointCollider, decltype(pt_inter_collision_filter)>(
            ob->point_colliders.value(), pt_inter_collision_filter, pt_ccache,
            pt_ccache_fill, rt);
    oa->edge_collider_tree
        .test_ext_collision<EdgeCollider, decltype(ee_inter_collision_filter)>(
            ob->edge_collider_tree.get_colliders(), ee_inter_collision_filter,
            ee_ccache, ee_ccache_fill, rt);

    if (pt_ccache_fill == pt_ccache.size()) {
      pt_narrowphase(pt_ccache, collisions, collision_fill, rt);
      pt_ccache_fill = 0;
    }
    if (ee_ccache_fill == ee_ccache.size()) {
      ee_narrowphase(ee_ccache, collisions, collision_fill, rt);
      ee_ccache_fill = 0;
    }
  }

  // Self-collision detection within individual objects.
  // Uses same BVH-tree + CCD approach but with stricter filtering.
  for (auto& o : object_colliders) {
    // Skip pure obstacles and objects with self-collision disabled.
    if (!o.is_physical || !o.is_self_collision_on) {
      continue;
    }

    o.triangle_collider_tree
        .test_ext_collision<PointCollider, decltype(pt_inter_collision_filter)>(
            o.point_colliders.value(), pt_inter_collision_filter, pt_ccache,
            pt_ccache_fill, rt);
    o.edge_collider_tree.test_self_collision(ee_self_collision_filter,
                                             ee_ccache, ee_ccache_fill, rt);

    if (pt_ccache_fill == pt_ccache.size()) {
      pt_narrowphase(pt_ccache, collisions, collision_fill, rt);
      pt_ccache_fill = 0;
    }
    if (ee_ccache_fill == ee_ccache.size()) {
      ee_narrowphase(ee_ccache, collisions, collision_fill, rt);
      ee_ccache_fill = 0;
    }
  }

  if (pt_ccache_fill != 0) {
    pt_narrowphase(pt_ccache, collisions, collision_fill, rt);
  }
  if (ee_ccache_fill != 0) {
    ee_narrowphase(ee_ccache, collisions, collision_fill, rt);
  }

  return collision_fill;
}

}  // namespace silk::cuda::collision
