#pragma once

#include <Eigen/Core>

#include "solver.hpp"

namespace silk {

class SolverBody {
 public:
  virtual ~SolverBody() = default;

  virtual int get_vert_num() const = 0;
  virtual Eigen::Ref<const Eigen::VectorXf> get_init_position() const = 0;
  virtual int get_position_offset() const = 0;
  virtual void set_position_offset(int offset) = 0;
  virtual Eigen::Ref<const Eigen::VectorXi> get_pinned_verts() const = 0;

  virtual SolverInitData compute_solver_init_data() const = 0;
};

}  // namespace silk
