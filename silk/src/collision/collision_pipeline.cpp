#include <algorithm>
#include <cassert>
#include <memory>
#include <vector>

#include "../bbox.hpp"
#include "ccd.hpp"
#include "collision.hpp"
#include "sap.hpp"
#include "sap_kd_tree.hpp"

namespace silk {

struct ObstacleCollider {
  Bbox bbox;
  IObstacle* obstacle;
  ObstacleStatus status;
  std::vector<MeshCollider> mesh_colliders;
  KDTree<MeshCollider> mesh_collider_tree;
};

class CollisionPipeline::CollisionPipelineImpl {
 private:
  std::vector<ObstacleCollider> obstacle_colliders_;
  CCDSolver ccd_solver_;

 public:
  void add_obstacle(IObstacle* obstacle) {
    assert(obstacle);

    ObstacleCollider c;
    c.obstacle = obstacle;
    c.status = obstacle->get_obstacle_status();
    c.mesh_colliders = obstacle->init_mesh_colliders();
    assert(!c.mesh_colliders.empty());
    c.mesh_collider_tree.init(c.mesh_colliders.data(), c.mesh_colliders.size());

    obstacle_colliders_.emplace_back(std::move(c));
  }

  void remove_obstacle(IObstacle* obstacle) {
    assert(obstacle);

    auto pred = [obstacle](const ObstacleCollider& c) -> bool {
      return obstacle == c.obstacle;
    };

    auto it = std::find_if(obstacle_colliders_.begin(),
                           obstacle_colliders_.end(), pred);
    assert((it != obstacle_colliders_.end()));

    std::swap(*it, obstacle_colliders_.back());
    obstacle_colliders_.pop_back();
  }

  void update_obstacles(const Eigen::VectorXf& position) {
    for (auto& c : obstacle_colliders_) {
      c.status = c.obstacle->get_obstacle_status();
      if (c.status.is_static) {
        return;
      }

      // if obstacle is not static, update mesh colliders and its kd tree
      c.obstacle->update_mesh_colliders(c.mesh_colliders, position);
      c.bbox = c.mesh_colliders[0].bbox;
      for (int i = 1; i < c.mesh_colliders.size(); ++i) {
        c.bbox.merge_inplace(c.mesh_colliders[i].bbox);
      }
      c.mesh_collider_tree.update(c.bbox);
    }
  }

  std::optional<PositionConstrain> get_position_constrain(
      const Eigen::VectorXf& position, const Eigen::VectorXf& prev_position,
      const ObstacleCollider& oa, const MeshCollider& ma,
      const ObstacleCollider& ob, const MeshCollider& mb) {
    // fetch position at t0 and t1 then do a ccd test
    if (ma.type == MeshColliderType::Edge) {
      auto v10 = prev_position(Eigen::seqN(ma.v1, 3));
      auto v20 = prev_position(Eigen::seqN(ma.v2, 3));
      auto v30 = prev_position(Eigen::seqN(mb.v1, 3));
      auto v40 = prev_position(Eigen::seqN(mb.v2, 3));
      auto v11 = position(Eigen::seqN(ma.v1, 3));
      auto v21 = position(Eigen::seqN(ma.v2, 3));
      auto v31 = position(Eigen::seqN(mb.v1, 3));
      auto v41 = position(Eigen::seqN(mb.v2, 3));

      auto toi = ccd_solver_.edge_edge_ccd(v10, v20, v30, v40, v11, v21, v31,
                                           v41, 0.0f, 1.0f);
    }

    if (ma.type == MeshColliderType::Point) {
      auto p0 = prev_position(Eigen::seqN(ma.v1, 3));
      auto v10 = prev_position(Eigen::seqN(mb.v1, 3));
      auto v20 = prev_position(Eigen::seqN(mb.v2, 3));
      auto v30 = prev_position(Eigen::seqN(mb.v3, 3));
      auto p1 = position(Eigen::seqN(ma.v1, 3));
      auto v11 = position(Eigen::seqN(mb.v1, 3));
      auto v21 = position(Eigen::seqN(mb.v2, 3));
      auto v31 = position(Eigen::seqN(mb.v3, 3));

      auto toi = ccd_solver_.point_triangle_ccd(p0, v10, v20, v30, p1, v11, v21,
                                                v31, 0.0f, 1.0f);
    }

    auto p0 = prev_position(Eigen::seqN(mb.v1, 3));
    auto v10 = prev_position(Eigen::seqN(ma.v1, 3));
    auto v20 = prev_position(Eigen::seqN(ma.v2, 3));
    auto v30 = prev_position(Eigen::seqN(ma.v3, 3));
    auto p1 = position(Eigen::seqN(mb.v1, 3));
    auto v11 = position(Eigen::seqN(ma.v1, 3));
    auto v21 = position(Eigen::seqN(ma.v2, 3));
    auto v31 = position(Eigen::seqN(ma.v3, 3));

    auto toi = ccd_solver_.point_triangle_ccd(p0, v10, v20, v30, p1, v11, v21,
                                              v31, 0.0f, 1.0f);
  }

  PositionConstrain resolve_collision(const Eigen::VectorXf& position,
                                      const Eigen::VectorXf& prev_position) {
    update_obstacles(position);

    std::vector<ObstacleCollider*> proxies(obstacle_colliders_.size());

    for (int i = 0; i < obstacle_colliders_.size(); ++i) {
      proxies[i] = obstacle_colliders_.data() + i;
    }

    CollisionFilter<ObstacleCollider> obstacle_filter =
        [](const ObstacleCollider& a, const ObstacleCollider& b) {
          const ObstacleStatus& sa = a.status;
          const ObstacleStatus& sb = b.status;

          // if group = -1, collision with others is disabled.
          // if groups are different, a and b does not collide.
          // if both a and b are pure collider, collision is meaningless.
          return (sa.group != -1 && sb.group != -1 && sa.group == sb.group &&
                  !(sa.is_pure_obstacle && sb.is_pure_obstacle));
        };
    CollisionCache<ObstacleCollider> obstacle_ccache;

    // at obstacle level, use sweep and prune to find collision
    int axis = sap_optimal_axis(proxies.data(), proxies.size());
    sap_sort_proxies(proxies.data(), proxies.size(), axis);
    sap_sorted_group_self_collision(proxies.data(), proxies.size(), axis,
                                    obstacle_filter, obstacle_ccache);

    // mesh level collision

    CollisionFilter<MeshCollider> dummy_filter =
        [](const MeshCollider& a, const MeshCollider& b) -> bool {
      return true;
    };
    CollisionCache<MeshCollider> mesh_ccache;

    // obstacle-obstacle collision
    for (auto& [a, b] : obstacle_ccache) {
      KDTree<MeshCollider>::test_tree_collision(a->mesh_collider_tree,
                                                b->mesh_collider_tree,
                                                dummy_filter, mesh_ccache);
    }

    // obstacle self collision
    CollisionFilter<MeshCollider> neighbor_filter =
        [](const MeshCollider& a, const MeshCollider& b) -> bool {
      // reject neighbors
      if (a.type == MeshColliderType::Point &&
          b.type == MeshColliderType::Triangle) {
        return (a.v1 != b.v1 && a.v1 != b.v2 && a.v1 != b.v2);
      } else if (a.type == MeshColliderType::Triangle &&
                 b.type == MeshColliderType::Point) {
        return (b.v1 != a.v1 && b.v1 != a.v2 && b.v1 != a.v2);
      } else if (a.type == MeshColliderType::Edge &&
                 b.type == MeshColliderType::Edge) {
        return (a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v1 && a.v2 != b.v2);
      }
      return false;
    };
    mesh_ccache.clear();
    for (auto& c : obstacle_colliders_) {
      if (!c.status.is_pure_obstacle && c.status.is_self_collision_on) {
        c.mesh_collider_tree.test_self_collision(neighbor_filter, mesh_ccache);
        // grep position
        // exact test
        // compute constrain
      }
    }
  }
};

void CollisionPipeline::add_obstacle(IObstacle* obstacle) {
  impl_->add_obstacle(obstacle);
}
void CollisionPipeline::remove_obstacle(IObstacle* obstacle) {
  impl_->add_obstacle(obstacle);
}
PositionConstrain CollisionPipeline::resolve_collision(
    const Eigen::VectorXf& position) {
  return impl_->resolve_collision(position);
}

}  // namespace silk
