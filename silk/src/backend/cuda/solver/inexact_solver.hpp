#pragma once

#include "backend/cuda/solver/cloth_solver_context.hpp"

namespace silk::cuda {

void inexact_solve(const ClothSolverContext& solver_context, const float* d_rhs,
                   float* d_x);

}  // namespace silk::cuda
