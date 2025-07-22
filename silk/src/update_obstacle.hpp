#pragma once

#include <Eigen/Core>

#include "ecs.hpp"

namespace silk {

void update_all_obstacles(Registry& registry,
                          const Eigen::VectorXf& solver_state,
                          const Eigen::VectorXf& prev_solver_state);

}  // namespace silk
