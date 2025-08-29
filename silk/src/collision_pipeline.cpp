#include "collision_pipeline.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <cassert>
#include <optional>
#include <tight_inclusion/interval_root_finder.hpp>
#include <vector>

#include "collision.hpp"
#include "collision_broadphase.hpp"
#include "collision_narrowphase.hpp"
#include "mesh_collider.hpp"
#include "object_collider.hpp"

namespace silk {

// for object collision.
bool object_collision_filter(const ObjectCollider& a, const ObjectCollider& b) {
  // if group = -1, collision with others is disabled.
  // if groups are different, a and b does not collide.
  // if both a and b are pure obstacle, collision is meaningless.
  return (a.group != -1 && b.group != -1 && a.group == b.group &&
          !(a.solver_offset == -1 && b.solver_offset == -1));
};

// for mesh collision between objects.
bool mesh_collision_filter(const MeshCollider& a, const MeshCollider& b) {
  return true;
};

// for mesh self collision.
bool mesh_self_collision_filter(const MeshCollider& a, const MeshCollider& b) {
  // 1. only accept point triangle or edge edge collision.
  // 2. reject collision between neighboring primitives.
  // 3. reject collision between pinnned primitives, which has 0 inverse mass.

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

std::vector<Collision> CollisionPipeline::find_collision(
    std::vector<ObjectCollider>& object_colliders, float dt) {
  // compute floating point err for ccd
  Eigen::Vector3f abs_max = Eigen::Vector3f::Zero();
  for (auto& o : object_colliders) {
    Eigen::Vector3f bbox_abs_min = o.bbox.min.cwiseAbs();
    Eigen::Vector3f bbox_abs_max = o.bbox.max.cwiseAbs();
    abs_max = (abs_max.cwiseMax(bbox_abs_min.cwiseMax(bbox_abs_max))).eval();
  }

  scene_ee_err_ = ticcd::get_numerical_error(abs_max, false, true);
  scene_vf_err_ = ticcd::get_numerical_error(abs_max, true, true);

  std::vector<std::vector<Collision>> thread_local_collisions(
      omp_get_max_threads());
  CollisionCache<MeshCollider> mesh_ccache;

  // find collisions between object colliders.
  // we will do:
  // 1. object collider broadphase using sweep and prune
  // 2. mesh collider broadphase using kd tree
  // 3. mesh collider narrowphase using ccd

  // step 1. object collider broadphase using sweep and prune
  CollisionCache<ObjectCollider> object_ccache;
  std::vector<ObjectCollider*> object_proxies(object_colliders.size());
  for (int i = 0; i < object_colliders.size(); ++i) {
    object_proxies[i] = object_colliders.data() + i;
  }
  int axis = sap_optimal_axis<ObjectCollider>(object_proxies.data(),
                                              object_proxies.size());
  sap_sort_proxies<ObjectCollider>(object_proxies.data(), object_proxies.size(),
                                   axis);
  sap_sorted_group_self_collision<ObjectCollider>(
      object_proxies.data(), object_proxies.size(), axis,
      object_collision_filter, object_ccache);

  for (auto& [oa, ob] : object_ccache) {
    // step 2. mesh collider broadphase using kd tree
    mesh_ccache.clear();
    KDTree<MeshCollider>::test_tree_collision(
        oa->mesh_collider_tree, ob->mesh_collider_tree, mesh_collision_filter,
        mesh_ccache);

// step 3. mesh collider narrowphase using ccd
#pragma omp parallel for
    for (auto& [ma, mb] : mesh_ccache) {
      auto collision = narrow_phase(*oa, *ma, *ob, *mb, dt,
                                    collision_stiffness_base, ccd_tolerance,
                                    ccd_max_iter, scene_ee_err_, scene_vf_err_);
      auto& out = thread_local_collisions[omp_get_thread_num()];
      if (collision) {
        out.emplace_back(std::move(*collision));
      }
    }
  }

  // find object collider self collision.
  // we will do:
  // 1. mesh collider broadphase using kd tree
  // 2. mesh collider narrowphase using ccd
  for (auto& o : object_colliders) {
    // skip if object is pure obstacle or self collision is off
    if (o.solver_offset == -1 || !o.is_self_collision_on) {
      continue;
    }

    // step 1. mesh collider broadphase using kd tree
    mesh_ccache.clear();
    o.mesh_collider_tree.test_self_collision(mesh_self_collision_filter,
                                             mesh_ccache);

#pragma omp parallel for
    for (auto& [ma, mb] : mesh_ccache) {
      // step 2. mesh collider narrowphase using ccd
      auto collision = narrow_phase(o, *ma, o, *mb, dt,
                                    collision_stiffness_base, ccd_tolerance,
                                    ccd_max_iter, scene_ee_err_, scene_vf_err_);
      auto& out = thread_local_collisions[omp_get_thread_num()];
      if (collision) {
        out.emplace_back(std::move(*collision));
      }
    }
  }

  auto& thread0_collisions = thread_local_collisions[0];
  for (int i = 1; i < omp_get_max_threads(); ++i) {
    auto& c = thread_local_collisions[i];
    thread0_collisions.insert(thread0_collisions.end(), c.begin(), c.end());
  }
  return thread0_collisions;
}

void CollisionPipeline::update_collision(const Eigen::VectorXf& solver_state_t0,
                                         const Eigen::VectorXf& solver_state_t1,
                                         std::vector<Collision>& collisions) {
  for (auto& c : collisions) {
    partial_ccd_update(solver_state_t0, solver_state_t1, scene_ee_err_,
                       scene_vf_err_, collision_stiffness_base,
                       collision_stiffness_max, collision_stiffness_growth,
                       ccd_tolerance, partial_ccd_max_iter, c);
  }
}

}  // namespace silk
