#pragma once

#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/mas_cg_solver.cuh"

namespace silk::cuda {

class ClothADMMHelper {
 private:
  // In-plane elastic aux state.
  Buf<float> ze_;
  Buf<float> ue_;
  // Bending aux state.
  Buf<float> zb_;
  Buf<float> ub_;
  // cusparse related.
  CuSparseHandle cusparse_handle_;
  Buf<char> cusparse_workspace_;
  // Other tmp.
  Buf<float> float_tmp_;

  MASCGSolver linear_solver_;

 public:
  ClothADMMHelper() = default;
  ClothADMMHelper(int vert_num, int face_num, CudaRuntime rt);

  void reset_aux_lagrange_mul(CudaRuntime rt);

  // clang-format off
  /// @brief Update aux variables z and lagrange multipliers. Accumulate primal/dual residual norms.
  /// @param[in] max_lagrange_mul         Clamp lagrange multipliers to this value.
  /// @param[in] l1_cache                 Precomputed assembly data.
  /// @param[in] state                    Current position vector.
  /// @param[out] primal_residual_norm2   ||W(Sx - z)||^2.
  /// @param[out] primal_scale_x2         ||W * Sx||^2.
  /// @param[out] primal_scale_aux2       ||W * z||^2.
  /// @param[out] dual_residual           rho * S^T W^T W (z_curr - z_prev).
  /// @param[out] dual_scale_curr         rho * S^T W^T W * z_curr.
  /// @param[out] dual_scale_prev         rho * S^T W^T W * z_prev.
  void update_aux_var_and_lagrange_mul(float max_lagrange_mul,
                                       const ClothAssemblyL1Cache& l1_cache,
                                       ctd::span<const float> state,
                                       ctd::span<float> primal_residual_norm2,
                                       ctd::span<float> primal_scale_x2,
                                       ctd::span<float> primal_scale_aux2,
                                       ctd::span<float> dual_residual,
                                       ctd::span<float> dual_scale_curr,
                                       ctd::span<float> dual_scale_prev,
                                       CudaRuntime rt);
  // clang-format on

  // clang-format off
  /// @brief Solve ADMM primal x.
  /// @param[in] rel_tol         Relative tolerance for linear solver.
  /// @param[in] abs_tol         Absolute tolerance for linear solver.
  /// @param[in] l1_cache        Precomputed assembly data.
  /// @param[in] is_lhs_changed  True if LHS matrix changed.
  /// @param[in] extern_lhs      External diagonal LHS contributions.
  /// @param[in] extern_rhs      External RHS contributions.
  /// @param[in] inertia_mod     Inertia term: -x_prev/dt^2 - v_prev/dt - acc.
  /// @param[in/out] state       Input is treated as initial guess.
  void solve_main_var(float rel_tol,
                      float abs_tol,
                      const ClothAssemblyL1Cache& l1_cache,
                      bool is_lhs_changed,
                      ctd::span<const float> extern_lhs,
                      ctd::span<const float> extern_rhs,
                      ctd::span<const float> inertia_mod,
                      ctd::span<float> state,
                      CudaRuntime rt);
  // clang-format on
};

}  // namespace silk::cuda
