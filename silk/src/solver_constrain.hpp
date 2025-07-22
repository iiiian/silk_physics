#pragma once

#include <Eigen/Core>

namespace silk {

class ISolverConstrain {
 public:
  virtual ~ISolverConstrain() = default;

  virtual void project(const Eigen::VectorXf& solver_state,
                       Eigen::VectorXf& out) const = 0;
};

}  // namespace silk
