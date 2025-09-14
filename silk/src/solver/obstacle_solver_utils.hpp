#pragma once

#include "../ecs.hpp"

namespace silk {

/** Reset object state and solver context to the start of simulation.
 */
void batch_reset_obstacle_simulation(Registry& registry);

/** Prepare obstacle for solver stepping at a given time step.
 *  Ensures `ObjectCollider` exists and are valid then update ObstaclePosition.
 *
 *  @param registry ECS storage for all components.
 *  @param entity ECS obstacle entity.
 */
void prepare_obstacle_simulation(Registry& registry, Entity& entity);

}  // namespace silk
