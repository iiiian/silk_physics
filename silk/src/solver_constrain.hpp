#pragma once

#include <Eigen/Core>

namespace silk {

class ISolverConstrain {
 public:
  virtual ~ISolverConstrain() = default;

  virtual void set_solver_state_offset(int offset) = 0;
  virtual void project(const Eigen::VectorXf& solver_state,
                       Eigen::VectorXf& out) const = 0;
};

}  // namespace silk
