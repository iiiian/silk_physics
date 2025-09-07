#pragma once

#include <Eigen/Core>

namespace silk {

struct BarrierConstrain {
  Eigen::VectorXf lhs;
  Eigen::VectorXf rhs;
};

}  // namespace silk
