#pragma once

#include <Eigen/Core>

namespace silk {

struct InitialState {
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;
};

}  // namespace silk
