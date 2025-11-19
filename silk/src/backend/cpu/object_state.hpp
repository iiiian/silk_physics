#pragma once

#include <Eigen/Core>

namespace silk::cpu {

struct ObjectState {
  int state_offset;
  int state_num;
  Eigen::VectorXf curr_state;
  Eigen::VectorXf state_velocity;
};

}  // namespace silk::cpu
