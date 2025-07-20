#include <cassert>
#include <vector>

#include "ccd.hpp"
#include "collision.hpp"
#include "sap.hpp"
#include "sap_kd_tree.hpp"

namespace silk {

std::optional<Collision> narrow_phase(const Obstacle& oa, const Collider& ca,
                                      const Obstacle& ob, const Collider& cb) {
  // edge edge collision
  if (ca.type == ColliderType::Edge) {
    // TODO: compute h
    auto impact = edge_edge_ccd(ca.v10, ca.v20, cb.v10, cb.v20, ca.v11, ca.v21,
                                cb.v11, cb.v21, 0.0f, 0.05f, 10, 1e-6f);
    if (impact) {
      Collision collision;
      collision.ca = ca;
      collision.cb = cb;
      collision.update_collider_a = oa.update_collider;
      collision.update_collider_b = ob.update_collider;
      collision.impact = std::move(*impact);
      return collision;
    } else {
      return std::nullopt;
    }
  }
  // point triangle collision: a is point and b is triangle
  else if (ca.type == ColliderType::Point) {
    auto impact =
        point_triangle_ccd(ca.v10, cb.v10, cb.v20, cb.v30, ca.v11, cb.v11,
                           cb.v21, cb.v31, 0.0f, 0.0f, 1, 0.0f);
    if (impact) {
      Collision collision;
      collision.ca = ca;
      collision.cb = cb;
      collision.update_collider_a = oa.update_collider;
      collision.update_collider_b = ob.update_collider;
      collision.impact = std::move(*impact);
      return collision;
    } else {
      return std::nullopt;
    }
  }
  // point triangle collision: a is triangle and b is point
  else {
    auto impact =
        point_triangle_ccd(cb.v10, ca.v10, ca.v20, ca.v30, cb.v11, ca.v11,
                           ca.v21, ca.v31, 0.0f, 0.0f, 1, 0.0f);

    if (impact) {
      Collision collision;
      collision.ca = cb;
      collision.cb = ca;
      collision.update_collider_a = ob.update_collider;
      collision.update_collider_b = oa.update_collider;
      collision.impact = std::move(*impact);
      return collision;
    } else {
      return std::nullopt;
    }
  }
}

CollisionCache<Obstacle> obstacle_broadphase(std::vector<Obstacle>& obstacles) {
  // prepare obstacle proxies for sweep and prune
  std::vector<Obstacle*> proxies(obstacles.size());
  for (int i = 0; i < obstacles.size(); ++i) {
    proxies[i] = obstacles.data() + i;
  }

  // if group = -1, collision with others is disabled.
  // if groups are different, a and b does not collide.
  // if both a and b are pure collider, collision is meaningless.
  CollisionFilterCallback<Obstacle> obstacle_filter = [](const Obstacle& a,
                                                         const Obstacle& b) {
    return (a.group != -1 && b.group != -1 && a.group == b.group &&
            !(a.is_pure_obstacle && b.is_pure_obstacle));
  };
  CollisionCache<Obstacle> ccache;

  // use sweep and prune to find collision
  int axis = sap_optimal_axis(proxies.data(), proxies.size());
  sap_sort_proxies(proxies.data(), proxies.size(), axis);
  sap_sorted_group_self_collision(proxies.data(), proxies.size(), axis,
                                  obstacle_filter, ccache);
  return ccache;
}

std::vector<Collision> find_collision(std::vector<Obstacle>& obstacles) {
  auto obstacle_ccache = obstacle_broadphase(obstacles);
  std::vector<Collision> collisions;

  // mesh level obstacle-obstacle collision
  CollisionFilterCallback<Collider> dummy_filter =
      [](const Collider& a, const Collider& b) -> bool { return true; };
  CollisionCache<Collider> collider_ccache;
  for (auto& [oa, ob] : obstacle_ccache) {
    // obstacle-obstacle collision broadphase
    KDTree<Collider>::test_tree_collision(oa->mesh_collider_tree,
                                          ob->mesh_collider_tree, dummy_filter,
                                          collider_ccache);
    // obstacle obstacle collision narrowphase
    for (auto& [ca, cb] : collider_ccache) {
      auto collision = narrow_phase(*oa, *ca, *ob, *cb);
      if (collision) {
        collisions.emplace_back(std::move(collision));
      }
    }
  }

  // mesh level obstacle self collision
  CollisionFilterCallback<Collider> neighbor_filter =
      [](const Collider& a, const Collider& b) -> bool {
    // reject neighbors
    if (a.type == ColliderType::Point && b.type == ColliderType::Triangle) {
      return (a.v1 != b.v1 && a.v1 != b.v2 && a.v1 != b.v2);
    } else if (a.type == ColliderType::Triangle &&
               b.type == ColliderType::Point) {
      return (b.v1 != a.v1 && b.v1 != a.v2 && b.v1 != a.v2);
    } else if (a.type == ColliderType::Edge && b.type == ColliderType::Edge) {
      return (a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v1 && a.v2 != b.v2);
    }
    return false;
  };
  for (auto& o : obstacles) {
    if (o.is_pure_obstacle || !o.is_self_collision_on) {
      continue;
    }

    // self collision broadphase
    collider_ccache.clear();
    o.mesh_collider_tree.test_self_collision(neighbor_filter, collider_ccache);
    // self collision narrowphase
    for (auto& [ca, cb] : collider_ccache) {
      auto collision = narrow_phase(o, *ca, o, *cb);
      if (collision) {
        collisions.emplace_back(std::move(collision));
      }
    }
  }

  return collisions;
}

}  // namespace silk
