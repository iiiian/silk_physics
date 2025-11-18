#pragma once

#include "backend/cuda/solver/cloth_solver_context.hpp"

namespace silk::cuda {

void inexact_solve(const ClothSolverContext& solver_context, const float* d_rhs,
                   const float* d_barrier_lhs, float* d_x);

}  // namespace silk::cuda
