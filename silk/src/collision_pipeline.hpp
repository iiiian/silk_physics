#pragma once

#include <vector>

#include "collision.hpp"
#include "object_collider.hpp"

namespace silk {

class CollisionPipeline {
 public:
  float toi_tolerance = 0.05;
  int toi_bisect_it = 4;
  float eps = 1e-6;

  std::vector<Collision> find_collision(
      std::vector<ObjectCollider>& object_colliders, float dt) const;
};

}  // namespace silk
