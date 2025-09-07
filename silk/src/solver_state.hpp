#pragma once

#include <Eigen/Core>

namespace silk {

struct SolverState {
  int state_offset;
  int state_num;
  Eigen::VectorXf curr_state;
  Eigen::VectorXf state_velocity;
};

}  // namespace silk
