#include "backend/cuda/collision/pipeline.hpp"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <cassert>
#include <vector>

#include "backend/cuda/collision/broadphase.hpp"
#include "backend/cuda/collision/collision.hpp"
#include "backend/cuda/collision/interval_root_finder.hpp"
#include "backend/cuda/collision/mesh_collider.hpp"
#include "backend/cuda/collision/narrowphase.hpp"
#include "backend/cuda/collision/object_collider.hpp"
#include "common/logger.hpp"

namespace silk::cuda {

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

std::vector<Collision> CollisionPipeline::find_collision(Registry& registry,
                                                         const Bbox& scene_bbox,
                                                         float dt) {
  // Compute scene-dependent numerical error bounds for CCD robustness.
  Eigen::Vector3f abs_max =
      scene_bbox.min.cwiseAbs().cwiseMax(scene_bbox.max.cwiseAbs());
  scene_ee_err_ = get_numerical_error(abs_max, false);
  scene_vf_err_ = get_numerical_error(abs_max, true);

  tbb::enumerable_thread_specific<std::vector<Collision>> thread_collisions;
  std::vector<ObjectCollider>& object_colliders =
      registry.get_all<ObjectCollider>();

  // Three-stage collision detection for inter-object collisions:
  // 1. Object-level broadphase using sweep-and-prune
  // 2. Mesh-level broadphase using KD-tree spatial queries
  // 3. Narrowphase using continuous collision detection

  // Stage 1: Object broadphase using sweep-and-prune.
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

    // Stage 2: Mesh broadphase using hierarchical KD-tree traversal.
    mesh_ccache_.clear();
    KDTree<MeshCollider>::test_tree_collision(
        oa->mesh_collider_tree, ob->mesh_collider_tree,
        mesh_inter_collision_filter, mesh_ccache_);

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

}  // namespace silk::cuda
