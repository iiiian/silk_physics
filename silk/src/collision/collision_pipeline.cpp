#include <algorithm>
#include <cassert>
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

  void resolve_collision() {
    update_obstacles();

    // prepare obstacle collider proxies for obstable level broadphase sap
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

    // use sweep and prune to find collision
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
    for (auto& [oa, ob] : obstacle_ccache) {
      // obstacle-obstacle collision broadphase
      KDTree<MeshCollider>::test_tree_collision(oa->mesh_collider_tree,
                                                ob->mesh_collider_tree,
                                                dummy_filter, mesh_ccache);
      // obstacle obstacle collision narrowphase
      for (auto& [ma, mb] : mesh_ccache) {
        narrow_phase(*oa, *ma, *ob, *mb);
      }
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
      if (c.status.is_pure_obstacle || !c.status.is_self_collision_on) {
        continue;
      }

      // self collision broadphase
      c.mesh_collider_tree.test_self_collision(neighbor_filter, mesh_ccache);
      // self collision narrowphase
      for (auto& [ma, mb] : mesh_ccache) {
        narrow_phase(c, *ma, c, *mb);
      }
    }
  }

 private:
  void update_obstacles() {
    for (auto& c : obstacle_colliders_) {
      c.status = c.obstacle->get_obstacle_status();
      if (c.status.is_static) {
        return;
      }

      // if obstacle is not static, update mesh colliders and its kd tree
      c.obstacle->update_mesh_colliders(c.mesh_colliders);
      c.bbox = c.mesh_colliders[0].bbox;
      for (int i = 1; i < c.mesh_colliders.size(); ++i) {
        c.bbox.merge_inplace(c.mesh_colliders[i].bbox);
      }
      c.mesh_collider_tree.update(c.bbox);
    }
  }

  void narrow_phase(const ObstacleCollider& oa, const MeshCollider& ma,
                    const ObstacleCollider& ob, const MeshCollider& mb) {
    MeshColliderDetail da = oa.obstacle->get_mesh_collider_detail(ma.index);
    MeshColliderDetail db = ob.obstacle->get_mesh_collider_detail(mb.index);

    // edge edge collision
    if (ma.type == MeshColliderType::Edge) {
      // TODO: compute h
      auto im = edge_edge_ccd(da.v10, da.v20, db.v10, db.v20, da.v11, da.v21,
                              db.v11, db.v21, 0.0f, 0.05f, 10, 1e-6f);
      MeshColliderImpact impact_a;
      impact_a.v1 = im->v1;
      impact_a.v2 = im->v2;
      impact_a.normal = im->normal;
      impact_a.index = ma.index;
      impact_a.toi = im->toi;
      impact_a.weight = db.w1 + db.w2;

      oa.obstacle->resolve_impact(std::move(impact_a));

      MeshColliderImpact impact_b;
      impact_b.v1 = im->v3;
      impact_b.v2 = im->v4;
      impact_b.normal = -im->normal;
      impact_b.index = mb.index;
      impact_b.toi = im->toi;
      impact_b.weight = da.w1 + da.w2;

      ob.obstacle->resolve_impact(std::move(impact_b));
    }
    // point triangle collision: a is point and b is triangle
    else if (ma.type == MeshColliderType::Point) {
      auto im = point_triangle_ccd(da.v10, db.v10, db.v20, db.v30, da.v11,
                                   db.v11, db.v21, db.v31, 0.0f, 0.0f, 1, 0.0f);

      MeshColliderImpact impact_a;
      impact_a.v1 = im->p;
      impact_a.normal = im->normal;
      impact_a.index = ma.index;
      impact_a.toi = im->toi;
      impact_a.weight = db.w1 + db.w2 + db.w3;
      ;

      oa.obstacle->resolve_impact(std::move(impact_a));

      MeshColliderImpact impact_b;
      impact_b.v1 = im->v1;
      impact_b.v2 = im->v2;
      impact_b.v3 = im->v3;
      impact_b.normal = -im->normal;
      impact_b.index = mb.index;
      impact_b.toi = im->toi;
      impact_b.weight = da.w1;

      ob.obstacle->resolve_impact(std::move(impact_b));
    }
    // point triangle collision: a is triangle and b is point
    else {
      auto im = point_triangle_ccd(db.v10, da.v10, da.v20, da.v30, db.v11,
                                   da.v11, da.v21, da.v31, 0.0f, 0.0f, 1, 0.0f);
      MeshColliderImpact impact_a;
      impact_a.v1 = im->v1;
      impact_a.v2 = im->v2;
      impact_a.v3 = im->v3;
      impact_a.normal = im->normal;
      impact_a.index = ma.index;
      impact_a.toi = im->toi;
      impact_a.weight = db.w1;

      oa.obstacle->resolve_impact(std::move(impact_a));

      MeshColliderImpact impact_b;
      impact_b.v1 = im->p;
      impact_b.normal = -im->normal;
      impact_b.index = mb.index;
      impact_b.toi = im->toi;
      impact_b.weight = da.w1 + da.w2 + da.w3;

      ob.obstacle->resolve_impact(std::move(impact_b));
    }
  }
};

// pimpl boilerplate
void CollisionPipeline::add_obstacle(IObstacle* obstacle) {
  impl_->add_obstacle(obstacle);
}
void CollisionPipeline::remove_obstacle(IObstacle* obstacle) {
  impl_->remove_obstacle(obstacle);
}
void CollisionPipeline::resolve_collision() { impl_->resolve_collision(); }

}  // namespace silk
