#pragma once

#include "ecs.hpp"

namespace silk {

// update or create object collider
void init_all_object_colliders(Registry& registry,
                               const Eigen::VectorXf& solver_state,
                               const Eigen::VectorXf& prev_solver_state);

}  // namespace silk
