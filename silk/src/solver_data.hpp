#pragma once

#include <Eigen/SparseCore>
#include <memory>
#include <vector>

#include "solver_constrain.hpp"

namespace silk {

struct SolverData {
  int state_offset;
  int state_num;
  Eigen::VectorXf mass;
  Eigen::SparseMatrix<float> weighted_AA;
  std::vector<std::unique_ptr<ISolverConstrain>> constrains;

  // Since by default copy ctor is available and Eigen::SparseMatrix lacks
  // noexcept move ctor, we have to explicitly delet copy ctor to avoid error
  // when used in containers like std::vector
  SolverData() = default;
  SolverData(SolverData&) = delete;
  SolverData(SolverData&&) = default;
  SolverData& operator=(SolverData&) = delete;
  SolverData& operator=(SolverData&&) = default;
};

}  // namespace silk
