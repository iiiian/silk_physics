#include "collision_pipeline.hpp"

#include <Eigen/Core>
#include <cassert>
#include <optional>
#include <vector>

#include "ccd.hpp"
#include "collision.hpp"
#include "collision_broadphase.hpp"
#include "mesh_collider.hpp"
#include "object_collider.hpp"

namespace silk {

std::optional<Collision> CollisionPipeline::narrow_phase(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt) const {
  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  float h = std::min(oa.bbox_padding, ob.bbox_padding);
  CCDConfig config = {dt, damping, friction, h, toi_tolerance, toi_refine_it,
                      eps};

  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  Eigen::Vector4f weight;

  // edge edge collision
  if (ma.type == MeshColliderType::Edge) {
    position_t0.block(0, 0, 3, 2) = ma.position_t0.block(0, 0, 3, 2);
    position_t0.block(0, 2, 3, 2) = mb.position_t0.block(0, 0, 3, 2);
    position_t1.block(0, 0, 3, 2) = ma.position_t1.block(0, 0, 3, 2);
    position_t1.block(0, 2, 3, 2) = mb.position_t1.block(0, 0, 3, 2);
    weight(Eigen::seqN(0, 2)) = ma.weight(Eigen::seqN(0, 2));
    weight(Eigen::seqN(2, 2)) = mb.weight(Eigen::seqN(0, 2));

    auto collision = edge_edge_ccd(position_t0, position_t1, weight, config);
    if (!collision) {
      return std::nullopt;
    }

    collision->offset(Eigen::seqN(0, 2)) = 3 * ma.index(Eigen::seqN(0, 2));
    collision->offset(Eigen::seqN(0, 2)).array() += oa.solver_offset;
    collision->offset(Eigen::seqN(2, 2)) = 3 * mb.index(Eigen::seqN(0, 2));
    collision->offset(Eigen::seqN(2, 2)).array() += ob.solver_offset;
    return collision;
  }
  // point triangle collision: a is point and b is triangle
  else if (ma.type == MeshColliderType::Point) {
    position_t0.block(0, 0, 3, 1) = ma.position_t0.block(0, 0, 3, 1);
    position_t0.block(0, 1, 3, 3) = mb.position_t0.block(0, 0, 3, 3);
    position_t1.block(0, 0, 3, 1) = ma.position_t1.block(0, 0, 3, 1);
    position_t1.block(0, 1, 3, 3) = mb.position_t1.block(0, 0, 3, 3);
    weight(Eigen::seqN(0, 1)) = ma.weight(Eigen::seqN(0, 1));
    weight(Eigen::seqN(1, 3)) = mb.weight(Eigen::seqN(0, 3));

    auto collision =
        point_triangle_ccd(position_t0, position_t1, weight, config);
    if (!collision) {
      return std::nullopt;
    }

    collision->offset(Eigen::seqN(0, 1)) = 3 * ma.index(Eigen::seqN(0, 1));
    collision->offset(Eigen::seqN(0, 1)).array() += oa.solver_offset;
    collision->offset(Eigen::seqN(1, 3)) = 3 * mb.index(Eigen::seqN(0, 3));
    collision->offset(Eigen::seqN(1, 3)).array() += ob.solver_offset;
    return collision;
  }
  // point triangle collision: a is triangle and b is point
  else {
    position_t0.block(0, 0, 3, 1) = mb.position_t0.block(0, 0, 3, 1);
    position_t0.block(0, 1, 3, 3) = ma.position_t0.block(0, 0, 3, 3);
    position_t1.block(0, 0, 3, 1) = mb.position_t1.block(0, 0, 3, 1);
    position_t1.block(0, 1, 3, 3) = ma.position_t1.block(0, 0, 3, 3);
    weight(Eigen::seqN(0, 1)) = mb.weight(Eigen::seqN(0, 1));
    weight(Eigen::seqN(1, 3)) = ma.weight(Eigen::seqN(0, 3));

    auto collision =
        point_triangle_ccd(position_t0, position_t1, weight, config);
    if (!collision) {
      return std::nullopt;
    }

    collision->offset(Eigen::seqN(0, 1)) = 3 * mb.index(Eigen::seqN(0, 1));
    collision->offset(Eigen::seqN(0, 1)).array() += ob.solver_offset;
    collision->offset(Eigen::seqN(1, 3)) = 3 * ma.index(Eigen::seqN(0, 3));
    collision->offset(Eigen::seqN(1, 3)).array() += oa.solver_offset;
    return collision;
  }
}

CollisionCache<ObjectCollider> CollisionPipeline::object_broadphase(
    std::vector<ObjectCollider>& obstacles) const {
  // prepare obstacle proxies for sweep and prune
  std::vector<ObjectCollider*> proxies(obstacles.size());
  for (int i = 0; i < obstacles.size(); ++i) {
    proxies[i] = obstacles.data() + i;
  }

  // if group = -1, collision with others is disabled.
  // if groups are different, a and b does not collide.
  // if both a and b are pure collider, collision is meaningless.
  CollisionFilterCallback<ObjectCollider> obstacle_filter =
      [](const ObjectCollider& a, const ObjectCollider& b) {
        return (a.group != -1 && b.group != -1 && a.group == b.group &&
                !(a.solver_offset == -1 && b.solver_offset == -1));
      };
  CollisionCache<ObjectCollider> ccache;

  // use sweep and prune to find collision
  int axis = sap_optimal_axis(proxies.data(), proxies.size());
  sap_sort_proxies(proxies.data(), proxies.size(), axis);
  sap_sorted_group_self_collision(proxies.data(), proxies.size(), axis,
                                  obstacle_filter, ccache);
  return ccache;
}

std::vector<Collision> CollisionPipeline::find_collision(
    std::vector<ObjectCollider>& obstacles, float dt) const {
  auto obstacle_ccache = object_broadphase(obstacles);
  std::vector<Collision> collisions;

  // mesh level obstacle-obstacle collision
  CollisionFilterCallback<MeshCollider> dummy_filter =
      [](const MeshCollider& a, const MeshCollider& b) -> bool { return true; };
  CollisionCache<MeshCollider> mesh_ccache;
  for (auto& [oa, ob] : obstacle_ccache) {
    // obstacle-obstacle collision broadphase
    KDTree<MeshCollider>::test_tree_collision(oa->mesh_collider_tree,
                                              ob->mesh_collider_tree,
                                              dummy_filter, mesh_ccache);
    // obstacle obstacle collision narrowphase
    for (auto& [ma, mb] : mesh_ccache) {
      auto collision = narrow_phase(*oa, *ma, *ob, *mb, dt);
      if (collision) {
        collisions.emplace_back(std::move(*collision));
      }
    }
  }

  // mesh level obstacle self collision
  CollisionFilterCallback<MeshCollider> neighbor_filter =
      [](const MeshCollider& a, const MeshCollider& b) -> bool {
    if (a.type == MeshColliderType::Point &&
        b.type == MeshColliderType::Triangle) {
      bool is_neighbor = (a.index(0) == b.index(0) ||
                          a.index(0) == b.index(1) || a.index(0) == b.index(2));
      bool is_both_pinned =
          (a.weight(0) + b.weight(0) + b.weight(1) + b.weight(2) == 0.0f);
      return (!is_neighbor && !is_both_pinned);
    } else if (a.type == MeshColliderType::Triangle &&
               b.type == MeshColliderType::Point) {
      bool is_neighbor = (b.index(0) == a.index(0) ||
                          b.index(0) == a.index(1) || b.index(0) == a.index(2));
      bool is_both_pinned =
          (a.weight(0) + a.weight(1) + a.weight(2) + b.weight(0) == 0.0f);
      return (!is_neighbor && !is_both_pinned);
    } else if (a.type == MeshColliderType::Edge &&
               b.type == MeshColliderType::Edge) {
      bool is_neighbor =
          (a.index(0) == b.index(0) || a.index(0) == b.index(1) ||
           a.index(1) == b.index(0) || a.index(1) == b.index(1));
      bool is_both_pinned =
          (a.weight(0) + a.weight(1) + b.weight(0) + b.weight(1) == 0.0f);
      return (!is_neighbor && !is_both_pinned);
    }
    return false;
  };
  for (auto& o : obstacles) {
    if (o.solver_offset == -1 || !o.is_self_collision_on) {
      continue;
    }

    // self collision broadphase
    mesh_ccache.clear();
    o.mesh_collider_tree.test_self_collision(neighbor_filter, mesh_ccache);

    // self collision narrowphase
    for (auto& [ma, mb] : mesh_ccache) {
      auto collision = narrow_phase(o, *ma, o, *mb, dt);
      if (collision) {
        collisions.emplace_back(std::move(*collision));
      }
    }
  }

  return collisions;
}

}  // namespace silk
