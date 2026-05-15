#pragma once

#include <cstdint>

#include "backend/cpu/ecs.hpp"

namespace silk::cpu {

/// @brief Reset obstacle solver state to initial conditions.
/// @param registry ECS registry storing obstacle-related components.
/// @return void
void batch_reset_obstacle_simulation(Registry& registry);

/// @brief Prepare an obstacle entity for solver stepping.
/// Ensures `ObjectCollider` exists and updates obstacle pose data.
/// @param registry ECS storage for all components.
/// @param entity ECS obstacle entity id being initialized.
/// @return void
void prepare_obstacle_simulation(Registry& registry, uint32_t entity);

}  // namespace silk::cpu
