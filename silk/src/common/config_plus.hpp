#pragma once

#include <silk/silk.hpp>

namespace silk {

struct CollisionConfigPlus {
  // Additional fields.
  bool is_updated = true;

  // Original config.
  bool is_collision_on = true;       ///< Enable collision
  bool is_self_collision_on = true;  ///< Enable self-collision
  int group = 0;                     ///< Collision group
  float restitution = 0.3f;          ///< Coefficient of restitution [0,1]
  float friction = 0.3f;             ///< Coefficient of friction [0,1]
};

}  // namespace silk
