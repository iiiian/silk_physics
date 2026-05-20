#pragma once

#include <optional>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda {

class ADMMSolver {
 public:
  enum class Error { Diverge };

  int max_inner_iteration = 100;
  float max_lagrange_mul = 1e8;
  float linear_abs_tol = 1e-7;
  float linear_rel_tol_min = 1e-6;
  float linear_rel_tol_max = 1e-3;
  float initial_linear_rel_tol = 1e-2;
  float non_linear_rel_tol = 1e-3;
  float non_linear_abs_tol = 1e-5;

  // clang-format off
  std::optional<Error> solve(ObjRegistry& registry,
                             ctd::span<const float> prev_state,
                             ctd::span<const float> prev_velocity,
                             ctd::span<float> inner_state,
                             EqualityConstraints& pin_constraints,
                             EqualityConstraints* barrier_constraints,
                             float dt,
                             Vec3f const_acceleration,
                             CudaRuntime rt);
  // clang-format on

 private:
  int cached_state_num_ = 0;
  Buf<float> lhs_diag_;
  Buf<float> rhs_;
  Buf<float> inertia_mod_;
  Buf<float> scalar_primal_norm2_;
  Buf<float> scalar_primal_scale_x2_;
  Buf<float> scalar_primal_scale_aux2_;
  Buf<float> dual_residual_;
  Buf<float> dual_scale_curr_;
  Buf<float> dual_scale_prev_;
  Buf<float> scalar_dual_norm2_;
  Buf<float> scalar_dual_scale_curr_norm2_;
  Buf<float> scalar_dual_scale_prev_norm2_;
};

}  // namespace silk::cuda
