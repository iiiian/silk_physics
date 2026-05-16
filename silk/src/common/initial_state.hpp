#pragma once

#include <Eigen/Core>

#include "common/eigen_alias.hpp"

namespace silk {

class InitialState {
 public:
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;

  InitialState() = default;
  InitialState(const RMatrixX3f& V);
};

}  // namespace silk
