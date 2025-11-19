#pragma once

#include <Eigen/Core>

namespace silk::cpu {

struct BarrierConstrain {
  Eigen::VectorXf lhs;
  Eigen::VectorXf rhs;
};

}  // namespace silk::cpu
