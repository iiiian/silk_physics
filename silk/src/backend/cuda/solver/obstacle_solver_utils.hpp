#pragma once

#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

/// @brief Reset obstacle solver state to initial conditions.
/// @param registry ECS registry storing obstacle-related components.
/// @return void
void batch_reset_obstacle_simulation(Registry& registry);

/// @brief Prepare an obstacle entity for solver stepping.
/// Ensures `ObjectCollider` exists and updates obstacle pose data.
/// @param registry ECS storage for all components.
/// @param entity ECS obstacle entity being initialized.
/// @return void
void prepare_obstacle_simulation(Registry& registry, Entity& entity);

}  // namespace silk::cuda
