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

struct ClothConfigPlus {
  // Additional fields.
  bool is_updated = true;

  // Original config.
  /// In-plane stretching resistance (higher = stiffer)
  float elastic_stiffness = 100.0f;
  /// Out-of-plane bending resistance (higher = stiffer)
  float bending_stiffness = 0.0001f;
  /// Mass density per unit area
  float density = 0.1f;
  /// Velocity damping factor [0,1] (higher = more damping)
  float damping = 0.01f;
};

}  // namespace silk
