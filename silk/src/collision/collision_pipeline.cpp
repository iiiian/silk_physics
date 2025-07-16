#include <algorithm>
#include <cassert>
#include <memory>
#include <vector>

#include "ccd.hpp"
#include "collision.hpp"
#include "sap.hpp"
#include "sap_kd_tree.hpp"

namespace silk {

struct ObstacleColliderData {
  IObstacle* obstacle;
  ObstacleStatus status;
  std::vector<MeshCollider> mesh_colliders;
  KDTree<MeshColliderData> mesh_collider_tree;
};

using ObstacleCollider = Collider<ObstacleColliderData>;

class CollisionPipeline::CollisionPipelineImpl {
 private:
  std::vector<ObstacleCollider> obstacle_colliders_;
  CCDSolver ccd_solver_;

 public:
  void add_obstacle(IObstacle* obstacle) {
    assert(obstacle);

    ObstacleColliderData data;
    data.obstacle = obstacle;
    data.status = obstacle->get_obstacle_status();
    data.mesh_colliders = obstacle->init_mesh_colliders();
    assert(!data.mesh_colliders.empty());
    data.mesh_collider_tree.init(data.mesh_colliders.data(),
                                 data.mesh_colliders.size());

    obstacle_colliders_.emplace_back(std::move(data), std::move(data));
  }

  void remove_obstacle(IObstacle* obstacle) {
    assert(obstacle);

    auto pred = [obstacle](const ObstacleCollider& collider) -> bool {
      return obstacle == collider.data.obstacle;
    };

    auto it = std::find_if(obstacle_colliders_.begin(),
                           obstacle_colliders_.end(), pred);
    assert((it != obstacle_colliders_.end()));

    std::swap(*it, obstacle_colliders_.back());
    obstacle_colliders_.pop_back();
  }

  void update_obstacles(const Eigen::VectorXf& position) {
    for (auto& collider : obstacle_colliders_) {
      ObstacleColliderData& data = collider.data;
      data.status = data.obstacle->get_obstacle_status();
      if (data.status.is_static) {
        return;
      }

      // if obstacle is not static, update mesh colliders and its kd tree
      data.obstacle->update_mesh_colliders(data.mesh_colliders, position);
      collider.bbox = data.mesh_colliders[0].bbox;
      for (int i = 1; i < data.mesh_colliders.size(); ++i) {
        collider.bbox.merge_inplace(data.mesh_colliders[i].bbox);
      }
      data.mesh_collider_tree->update(collider.bbox);
    }
  }

  PositionConstrain resolve_collision(const Eigen::VectorXf& position) {
    update_obstacles(position);

    std::vector<ColliderProxy<ObstacleColliderData>> proxies(
        obstacle_colliders_.size());

    for (int i = 0; i < obstacle_colliders_.size(); ++i) {
      proxies[i] = obstacle_colliders_.data() + i;
    }

    CollisionFilter<ObstacleColliderData> obstacle_filter =
        [](const ObstacleColliderData& a, const ObstacleColliderData& b) {
          const ObstacleStatus& sa = a.status;
          const ObstacleStatus& sb = b.status;

          // if group = -1, collision with others is disabled.
          // if groups are different, a and b does not collide.
          // if both a and b are pure collider, collision is meaningless.
          return (sa.group != -1 && sb.group != -1 && sa.group == sb.group &&
                  !(sa.is_pure_obstacle && sb.is_pure_obstacle));
        };
    CollisionCache<ObstacleColliderData> obstacle_ccache;

    // at obstacle level, use sweep and prune to find collision
    int axis = sap_optimal_axis(proxies.data(), proxies.size());
    sap_sort_proxies(proxies.data(), proxies.size(), axis);
    sap_sorted_group_self_collision(proxies.data(), proxies.size(), axis,
                                    obstacle_filter, obstacle_ccache);

    // mesh level collision

    CollisionFilter<MeshColliderData> mesh_filter =
        [](const MeshColliderData& a, const MeshColliderData& b) -> bool {
      // reject neighbors
      if (a.type == MeshColliderType::POINT &&
          b.type == MeshColliderType::TRIANGLE) {
        return (a.v1 != b.v1 && a.v1 != b.v2 && a.v1 != b.v2);
      } else if (a.type == MeshColliderType::TRIANGLE &&
                 b.type == MeshColliderType::POINT) {
        return (b.v1 != a.v1 && b.v1 != a.v2 && b.v1 != a.v2);
      } else if (a.type == MeshColliderType::EDGE &&
                 b.type == MeshColliderType::EDGE) {
        return (a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v1 && a.v2 != b.v2);
      }
      return false;
    };
    CollisionCache<MeshColliderData> mesh_ccache;

    // obstacle-obstacle collision
    for (auto& [a, b] : obstacle_ccache) {
      KDTree<ObstacleColliderData>::test_tree_collision(
          *a.mesh_collider_tree, b.mesh_collider_tree, mesh_filter,
          mesh_ccache);
    }

    // obstacle self collision
    for (auto& collider : obstacle_colliders_) {
      ObstacleStatus& stat = collider.data.status;
      if (!stat.is_pure_obstacle && stat.is_self_collision_on) {
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
