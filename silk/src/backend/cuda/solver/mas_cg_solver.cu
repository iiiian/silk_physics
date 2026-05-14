#include "backend/cuda/solver/mas_cg_solver.cuh"

#include <chrono>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/span>

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/inner_product.cuh"
#include "backend/cuda/solver/mas_preconditioner.cuh"
#include "common/logger.hpp"

namespace silk::cuda::solver {

namespace {

using clock = std::chrono::steady_clock;

float elapsed(const std::chrono::time_point<clock> &begin) {
  return std::chrono::duration<float>(clock::now() - begin).count();
}

/// Device scalar devision num / denom.
/// Defaults to zero when denom is small because we only check termination
/// condition every 10 PCG iterations and beta should be zero if we already
/// arrive at solution.
void scalar_division(ctd::span<const float> num, ctd::span<const float> denom,
                     ctd::span<float> out, CudaRuntime rt) {
  auto op = [num, denom, out] __device__(int) {
    out[0] = (ctd::abs(denom[0]) < 1e-20) ? 0.0 : (num[0] / denom[0]);
  };
  cub::DeviceFor::Bulk(1, op, rt.stream.get());
}

/// Compute alpha * x + beta * y.
/// @params h_alpha Host alpha.
/// @params d_alpha Device alpha. nullptr implies 1.0.
/// @params h_beta Host beta.
/// @params d_beta Device beta. nullptr implies 1.0.
/// @params x Device vector x.
/// @params y Device vector y and the output.
/// @params rt Cuda runtime.
void axpby(float h_alpha, const float *d_alpha, float h_beta,
           const float *d_beta, ctd::span<const float> x, ctd::span<float> y,
           CudaRuntime rt) {
  auto op = [h_alpha, d_alpha, h_beta, d_beta, x, y] __device__(int idx) {
    float alpha = h_alpha * ((d_alpha == nullptr) ? 1.0 : *d_alpha);
    float beta = h_beta * ((d_beta == nullptr) ? 1.0 : *d_beta);
    y[idx] = alpha * x[idx] + beta * y[idx];
  };
  cub::DeviceFor::Bulk(x.size(), op, rt.stream.get());
}

}  // namespace

void MASCGSolver::setup_cusparse(BSRView A, CudaRuntime rt) {
  cusparse_A_ = CuSparseBSR(A);

  float alpha = 1.0;
  float beta = 0.0;
  // As long as size match, x and y doesnt matters.
  CuSparseConstVec x(*r_);
  CuSparseVec y(*z_);
  size_t workspace_size = 0;

  cusparseSetStream(cusparse_handle_.raw, rt.stream.get());
  // clang-format off
    cusparseSpMV_bufferSize(cusparse_handle_.raw,
                            CUSPARSE_OPERATION_NON_TRANSPOSE,
                            &alpha,
                            cusparse_A_.raw,
                            x.raw,
                            &beta,
                            y.raw,
                            CUDA_R_32F,
                            CUSPARSE_SPMV_ALG_DEFAULT,
                            &workspace_size);
  // clang-format on

  spmv_workspace_ = alloc<char>(rt, workspace_size);
}

void MASCGSolver::spmv(ctd::span<const float> x, ctd::span<float> y,
                       CudaRuntime rt) {
  float alpha = 1.0;
  float beta = 0.0;
  CuSparseConstVec x_desc(x);
  CuSparseVec y_desc(y);

  cusparseSetStream(cusparse_handle_.raw, rt.stream.get());
  // clang-format off
    cusparseSpMV(cusparse_handle_.raw,
                 CUSPARSE_OPERATION_NON_TRANSPOSE,
                 &alpha,
                 cusparse_A_.raw,
                 x_desc.raw,
                 &beta,
                 y_desc.raw,
                 CUDA_R_32F,
                 CUSPARSE_SPMV_ALG_DEFAULT,
                 spmv_workspace_->data());
  // clang-format on
}

void MASCGSolver::factorize(BSRView A, ctd::span<const int> part_offset,
                            CudaRuntime rt) {
  assert(A.dim != 0);

  auto total_begin = clock::now();

  // Initialize MAS.
  auto phase_begin = clock::now();
  mas_precond_.factorize(A, part_offset, rt);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_mas] [{:.6f}]", elapsed(phase_begin));

  phase_begin = clock::now();

  fine_dim_ = A.dim * A.block_dim;
  r_ = alloc<float>(rt, fine_dim_);
  p_ = alloc<float>(rt, fine_dim_);
  z_ = alloc<float>(rt, fine_dim_);
  Ap_ = alloc<float>(rt, fine_dim_);

  scalar_rz_ = alloc<float>(rt, 1);
  scalar_pAp_ = alloc<float>(rt, 1);
  scalar_alpha_ = alloc<float>(rt, 1);
  scalar_beta_ = alloc<float>(rt, 1);
  scalar_rz_old_ = alloc<float>(rt, 1);
  scalar_rr_ = alloc<float>(rt, 1);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_device_buffers] [{:.6f}]", elapsed(phase_begin));

  // Allocates buffers for CuSparse.
  phase_begin = clock::now();
  setup_cusparse(A, rt);
  rt.stream.sync();

  SPDLOG_TRACE("[factorize_cusparse] [{:.6f}]", elapsed(phase_begin));
  SPDLOG_TRACE("[factorize_total] [{:.6f}]", elapsed(total_begin));
}

/// https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf
MASCGSolver::Status MASCGSolver::solve(ctd::span<const float> b,
                                       ctd::span<float> x, CudaRuntime rt) {
  assert(b.size() == fine_dim_);
  assert(x.size() == fine_dim_);

  // Compute initial residual r = b-Ax.
  spmv(x, *r_, rt);
  axpby(1.0, nullptr, -1.0, nullptr, b, *r_, rt);

  // Compute z = M^-1 r.
  mas_precond_.apply(*r_, *z_, rt);
  // Initial search direction p = z;
  cu::copy_bytes(rt.stream, *z_, *p_);

  // Compute rz = r^T M^-1 r.
  inner_product(*r_, *z_, *scalar_rz_, rt);
  float rz0 = scalar_load(scalar_rz_->data(), rt);
  if (ctd::isnan(rz0) || !ctd::isfinite(rz0)) {
    return Status::InvalidInitialResidual;
  }

  // Compute rr = r^T r.
  float rr0 = 0.0;
  if (!use_preconditioned_residual_norm_) {
    inner_product(*r_, *r_, *scalar_rr_, rt);
    rr0 = scalar_load(scalar_rr_->data(), rt);
  }

  Status status = Status::ReachAbsTol;
  auto iter_window_begin = clock::now();
  for (int k = 1; k <= max_iter_; ++k) {
    // Compute Ap = A p.
    spmv(*p_, *Ap_, rt);
    // Compute pAp = p^T * A * p.
    inner_product(*p_, *Ap_, *scalar_pAp_, rt);
    // Compute alpha = (r M^-1 r) / (p^T A p).
    scalar_division(*scalar_rz_, *scalar_pAp_, *scalar_alpha_, rt);
    // Compute x = x + alpha A p.
    axpby(1.0, scalar_alpha_->data(), 1.0, nullptr, *p_, x, rt);

    // Compute residual b-Ax directly.
    if (k % true_residual_period_ == 0) {
      spmv(x, *r_, rt);
      axpby(1.0, nullptr, -1.0, nullptr, b, *r_, rt);
    }
    // Compute residual update using r' = r - alpha A p.
    // This saves one spmv but accumulates floating point error overtime.
    else {
      axpby(-1.0, scalar_alpha_->data(), 1.0, nullptr, *Ap_, *r_, rt);
    }

    // Compute z = M^-1 r.
    mas_precond_.apply(*r_, *z_, rt);

    // Compute rz = r M^-1 r.
    cu::copy_bytes(rt.stream, *scalar_rz_, *scalar_rz_old_);
    inner_product(*r_, *z_, *scalar_rz_, rt);

    iterations_ = k;

    // Check convergence every 10 iterations.
    if (k % 10 == 0) {
      if (use_preconditioned_residual_norm_) {
        float rz_new = scalar_load(scalar_rz_->data(), rt);
        residual_norm_ = ctd::sqrt(rz_new);
        if (rz_new <= rel_tol_ * rel_tol_ * rz0 ||
            rz_new <= abs_tol_ * abs_tol_) {
          status = (rz_new <= abs_tol_ * abs_tol_) ? Status::ReachAbsTol
                                                   : Status::ReachRelTol;
          break;
        }
      } else {
        inner_product(*r_, *r_, *scalar_rr_, rt);
        float rr = scalar_load(scalar_rr_->data(), rt);
        residual_norm_ = ctd::sqrt(rr);
        if (rr <= rel_tol_ * rel_tol_ * rr0 || rr <= abs_tol_ * abs_tol_) {
          status = (rr <= abs_tol_ * abs_tol_) ? Status::ReachAbsTol
                                               : Status::ReachRelTol;
          break;
        }
      }
    }

    // debug logging
    if (k % 100 == 0) {
      rt.stream.sync();
      SPDLOG_TRACE("[pcg_loop] [{:.6f}] [iter={}] [residual={:.6e}]",
                   elapsed(iter_window_begin), iterations_, residual_norm_);
      iter_window_begin = clock::now();
    }

    // Compute beta = rz / rz_old.
    scalar_division(*scalar_rz_, *scalar_rz_old_, *scalar_beta_, rt);
    // Compute direction update p' = M^-1 r + beta p.
    axpby(1.0, nullptr, 1.0, scalar_beta_->data(), *z_, *p_, rt);
  }

  if (iterations_ == max_iter_) {
    status = Status::ReachMaxIter;
  }

  SPDLOG_TRACE("[pcg_loop] [{:.6f}] [iter={}] [residual={:.6e}]",
               elapsed(iter_window_begin), iterations_, residual_norm_);

  return status;
}

}  // namespace silk::cuda::solver
