#pragma once

#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/mas_cg_solver.cuh"

namespace silk::cuda {

class ClothADMMHelper {
 private:
  // y := in-plane elastic aux state.
  Buf<float> y_;
  Buf<float> uy_;
  // z := bending aux state.
  Buf<float> z_;
  Buf<float> uz_;
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

  /// @brief Update ADMM aux variables and it's lagrange multipliers.
  /// @param max_lagrange_mul Max abs value of lagrange multipliers.
  /// @param l1_cache
  /// @param state
  /// @param primal_residual_norm2
  /// @param primal_scale_x2
  /// @param primal_scale_aux2
  /// @param dual_residual
  /// @param dual_scale_curr
  /// @param dual_scale_prev
  /// @param rt
  // clang-format off
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

  /// @brief Solve main ADMM variables.
  /// @param rel_tol
  /// @param abs_tol
  /// @param l1_cache
  /// @param is_lhs_changed
  /// @param extern_lhs
  /// @param extern_rhs
  /// @param inertia_mod
  /// @param state
  /// @param rt
  // clang-format off
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
