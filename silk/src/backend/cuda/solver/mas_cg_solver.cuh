#pragma once

#include <cuda/std/span>

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/mas_preconditioner.cuh"

namespace silk::cuda {

class MASCGSolver {
 public:
  enum class Status {
    ReachAbsTol,
    ReachRelTol,
    ReachMaxIter,
    InvalidInitialResidual,
  };

  int max_iter = 1e3;            //< Max iterations.
  int true_residual_period = 1;  //< Iterations to compute true residual b-Ax.
  float abs_tol = 1e-20;         //< Absolute tolerance.
  float rel_tol = 1e-4;          //< Relative tolerance.
  bool use_preconditioned_residual_norm = false;

 private:
  int fine_dim_ = 0;  //< A.dim * A.block_dim.
  int iterations_ = 0;
  float residual_norm_ = 0.0;

  MASPreconditioner mas_precond_;

  CuSparseHandle cusparse_handle_;
  Buf<char> spmv_workspace_;
  CuSparseBSR cusparse_A_;       //< Static part of A.
  ctd::span<const float> diag_;  //< Dynamic diagonal update of A.

  // PCG variables.
  Buf<float> r_;
  Buf<float> p_;
  Buf<float> z_;
  Buf<float> Ap_;
  Buf<char> reduction_storage_;
  Buf<float> scalar_rz_;
  Buf<float> scalar_pAp_;
  Buf<float> scalar_alpha_;
  Buf<float> scalar_beta_;
  Buf<float> scalar_rz_old_;
  Buf<float> scalar_rr_;

 public:
  /// @brief Init solver. Must be called before solve.
  /// @param A Input matrix and scalar diagonal update.
  /// @param part_offset CSR mesh partition offset.
  void factorize(DynamicBSRView A, ctd::span<const int> part_offset,
                 CudaRuntime rt);

  /// @brief Solve linear system Ax=b.
  /// https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf
  Status solve(ctd::span<const float> b, ctd::span<float> x, CudaRuntime rt);

 private:
  void setup_cusparse(BSRView A, CudaRuntime rt);

  void spmv(ctd::span<const float> x, ctd::span<float> y, CudaRuntime rt);
};

}  // namespace silk::cuda
