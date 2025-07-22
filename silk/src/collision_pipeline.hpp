#pragma once

#include <optional>
#include <vector>

#include "collision.hpp"
#include "mesh_collider.hpp"
#include "obstacle.hpp"

namespace silk {

class CollisionPipeline {
 public:
  float toi_tolerance;
  int toi_refine_it;
  float eps;

  std::vector<Collision> find_collision(std::vector<Obstacle>& obstacles,
                                        float dt) const;

 private:
  std::optional<Collision> narrow_phase(const Obstacle& oa,
                                        const MeshCollider& ma,
                                        const Obstacle& ob,
                                        const MeshCollider& mb, float dt) const;
  CollisionCache<Obstacle> obstacle_broadphase(
      std::vector<Obstacle>& obstacles) const;
};

}  // namespace silk
