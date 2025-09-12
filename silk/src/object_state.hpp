#pragma once

#include <Eigen/Core>

namespace silk {

struct ObjectState {
  // The range of this object in the global state array.
  int state_offset;
  int state_num;

  Eigen::VectorXf curr_state;
  Eigen::VectorXf state_velocity;
};

}  // namespace silk
