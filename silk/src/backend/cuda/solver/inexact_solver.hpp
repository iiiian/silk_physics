#pragma once

#include "backend/cuda/solver/barrier_constrain.hpp"
#include "backend/cuda/solver/cloth_solver_context.hpp"

namespace silk::cuda {

void inexact_solve(const ClothSolverContext& solver_context, const float* d_rhs,
                   const BarrierConstrain& barrier_constrain, int state_offset,
                   float* d_x);

}  // namespace silk::cuda
