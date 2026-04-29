#include <cassert>

#include "backend/cpu/collision/broadphase.hpp"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/collision/ccd.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/collision/pipeline.cuh"
#include "backend/cuda/simple_linalg.cuh"
#include "common/logger.hpp"

namespace silk::cuda::collision {

__both__ bool object_collision_filter(const ObjectCollider& a,
                                      const ObjectCollider& b) {
  // Collision rules:
  // - Group = -1 means collision is disabled completely
  // - Objects must belong to the same collision group otherwise
  // - At least one object must be physical (not pure obstacle)
  return (a.group != -1 && b.group != -1 && a.group == b.group &&
          !(a.state_offset == -1 && b.state_offset == -1));
};

__both__ bool tp_inter_collision_filter(const TriangleCollider& a,
                                        const PointCollider& b) {
  bool is_both_pinned =
      ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
  return !is_both_pinned;
};

__both__ bool tp_self_collision_filter(const TriangleCollider& a,
                                       const PointCollider& b) {
  bool is_both_pinned =
      ((b.inv_mass + a.inv_mass(0) + a.inv_mass(1) + a.inv_mass(2)) == 0.0f);
  bool is_neighbor =                 ba.index == a.index(2));

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

cu::device_buffer<Collision> CollisionPipeline::find_collision(
    Registry& registry, const Bbox& scene_bbox, float dt, CudaRuntime rt) {
  std::vector<ObjectCollider>& object_colliders =
      registry.get_all<ObjectCollider>();

  // Three-stage collision detection for inter-object collisions:
  // 1. Object-level broadphase using sweep-and-prune on CPU.
  // 2. Mesh-level broadphase using BVH-tree spatial queries on GPU.
  // 3. Narrowphase using continuous collision detection on GPU.

  // Stage 1: Object broadphase using sweep-and-prune.
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

  auto tp_ccache = cu::make_buffer<ctd::pair<TriangleCollider, PointCollider>>(
      rt.stream, rt.mr, 1000, cu::no_init);
  auto ee_ccache = cu::make_buffer<ctd::pair<EdgeCollider, EdgeCollider>>(
      rt.stream, rt.mr, 1000, cu::no_init);
  for (auto& [oa, ob] : object_ccache) {
    // Stage 2: Mesh broadphase using GPU BVH queries.
    oa->triangle_collider_tree.test_ext_collision(
        ob->point_colliders, tp_inter_collision_filter, tp_ccache, rt);
    oa->edge_collider_tree.test_ext_collision(
        ob->edge_collider_tree.get_colliders(), ee_inter_collision_filter,
        ee_ccache, rt);

    // Stage 3: Parallel narrowphase using continuous collision detection.
    int ccache_num = mesh_ccache_.size();
    tbb::parallel_for(0, ccache_num, [&](int i) {
      auto& pair = mesh_ccache_[i];
      auto* ma = pair.first;
      auto* mb = pair.second;

      auto collision = narrow_phase(
          *oa, *ma, *ob, *mb, dt, collision_stiffness_base, min_toi,
          ccd_tolerance, ccd_max_iter, scene_ee_err_, scene_vf_err_);
      auto& out = thread_collisions.local();
      if (collision) {
        out.emplace_back(std::move(*collision));
      }
    });
  }

  // Self-collision detection within individual objects.
  // Uses same KD-tree + CCD approach but with stricter filtering.
  for (auto& o : object_colliders) {
    // Skip pure obstacles and objects with self-collision disabled.
    if (o.state_offset == -1 || !o.is_self_collision_on) {
      continue;
    }

    // Self-collision broadphase using internal KD-tree traversal.
    mesh_ccache_.clear();
    o.mesh_collider_tree.test_self_collision(mesh_self_collision_filter,
                                             mesh_ccache_);

    int ccache_num = mesh_ccache_.size();
    tbb::parallel_for(0, ccache_num, [&](int i) {
      auto& pair = mesh_ccache_[i];
      auto* ma = pair.first;
      auto* mb = pair.second;

      // Self-collision narrowphase using same CCD algorithm.
      auto collision = narrow_phase(
          o, *ma, o, *mb, dt, collision_stiffness_base, min_toi, ccd_tolerance,
          ccd_max_iter, scene_ee_err_, scene_vf_err_);
      auto& out = thread_collisions.local();
      if (collision) {
        out.push_back(std::move(*collision));
      }
    });
  }

  // Merge thread-local collision lists into single result vector.
  std::vector<Collision> collisions;
  for (auto& c : thread_collisions) {
    collisions.insert(collisions.end(), c.begin(), c.end());
  }
  return collisions;
}

void CollisionPipeline::update_collision(
    const Eigen::VectorXf& global_state_t0,
    const Eigen::VectorXf& global_state_t1,
    std::vector<Collision>& collisions) const {
  for (auto& c : collisions) {
    partial_ccd_update(global_state_t0, global_state_t1, scene_ee_err_,
                       scene_vf_err_, collision_stiffness_base,
                       collision_stiffness_max, collision_stiffness_growth,
                       ccd_tolerance, partial_ccd_max_iter, c);
  }
}

}  // namespace silk::cuda::collision
