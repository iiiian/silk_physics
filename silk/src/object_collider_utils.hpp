#pragma once

#include "ecs.hpp"

namespace silk {

void init_all_object_collider(Registry& registry);
void update_all_physical_object_collider(
    Registry& registry, const Eigen::VectorXf& solver_state,
    const Eigen::VectorXf& prev_solver_state);
void update_all_obstacle_object_collider(Registry& registry);

}  // namespace silk
