#pragma once

#include <Eigen/SparseCore>
#include <memory>
#include <vector>

#include "solver_constrain.hpp"

namespace silk {

struct SolverData {
  int offset;
  Eigen::VectorXf mass;
  std::vector<Eigen::Triplet<float>> weighted_AA;
  std::vector<std::unique_ptr<ISolverConstrain>> constrains;
};

}  // namespace silk
