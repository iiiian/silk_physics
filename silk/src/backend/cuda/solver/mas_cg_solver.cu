// #include "MASSolver.hpp"

#include <Eigen/Core>
#include <chrono>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/std/cmath>
#include <cuda/std/optional>
#include <cuda/std/span>
#include <cuda/stream>
#include <stdexcept>

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/inner_product.cuh"
#include "backend/cuda/solver/mas_preconditioner.cuh"
#include "common/logger.hpp"

namespace silk::cuda::solver {

namespace {

using clock = std::chrono::steady_clock;

float elapsed_seconds(const std::chrono::time_point<clock> &begin) {
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

class MASCGSolver {
 public:
  int block_dim_ = 3;  ///< BSR block dim.
  int max_iter_ = 1e5;
  int true_residual_period_ = 4;
  float abs_tol_ = 1e-20;
  float rel_tol_ = 1e-6;
  bool use_preconditioned_residual_norm_ = false;

  int dim_ = 0;           ///< Input matrix A dim.
  int permuted_dim_ = 0;  ///< Dim with block padding.
  int iterations_ = 0;
  float residual_norm_ = 0.0;

  CuSparseHandle cusparse_handle_;

  MASPreconditioner mas_precond_;
  CuSparseBSR sparse_A_;      ///< CuSparse handle of A.
  Buf<char> spmv_workspace_;  ///< CuSparse temp.

  // PCG variables.
  Buf<float> x_;
  Buf<float> b_;
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
  // Allocate cusparse workspace buffer.
  void setup_cusparse(BSRView A, CudaRuntime rt) {
    sparse_A_ = CuSparseBSR(A);

    float alpha = 1.0;
    float beta = 0.0;
    CuSparseConstVec x_desc(*x_);
    CuSparseVec y_desc(*Ap_);
    size_t workspace_size = 0;

    cusparseSetStream(cusparse_handle_.raw, rt.stream.get());
    cusparseSpMV_bufferSize(
        cusparse_handle_.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
        sparse_A_.raw, x_desc.raw, &beta, y_desc.raw, CUDA_R_32F,
        CUSPARSE_SPMV_ALG_DEFAULT, &workspace_size);

    spmv_workspace_ = alloc<char>(rt, workspace_size);
  }

  void spmv(ctd::span<const float> x, ctd::span<float> y, CudaRuntime rt) {
    float alpha = 1.0;
    float beta = 0.0;
    CuSparseConstVec x_desc(x);
    CuSparseVec y_desc(y);

    cusparseSetStream(cusparse_handle_.raw, rt.stream.get());
    cusparseSpMV(cusparse_handle_.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
                 sparse_A_.raw, x_desc.raw, &beta, y_desc.raw, CUDA_R_32F,
                 CUSPARSE_SPMV_ALG_DEFAULT, spmv_workspace_->data());
  }

  void factorize(BSRView A, ctd::span<const int> part_offset, CudaRuntime rt) {
    auto total_begin = clock::now();
    int block_n = div_round_up(A.dim, block_dim_);

    // We do:
    // 1. K-way graph parition.
    // 2. Build permutation based on partition.
    // 3. Initialize MAS preconditioner.
    // 4. Allocates buffer for PCG loop.

    dim_ = A.dim;

    // Initialize MAS.
    auto phase_begin = clock::now();
    // TODO: pass part offset.
    mas_precond_.factorize(A, part_offset, rt);
    rt.stream.sync();
    SPDLOG_TRACE("[MAS] [factorize_mas] [{:.6f}]",
                 elapsed_seconds(phase_begin));

    phase_begin = clock::now();

    x_ = alloc<float>(rt, permuted_dim_);
    b_ = alloc<float>(rt, permuted_dim_);
    r_ = alloc<float>(rt, permuted_dim_);
    p_ = alloc<float>(rt, permuted_dim_);
    z_ = alloc<float>(rt, permuted_dim_);
    Ap_ = alloc<float>(rt, permuted_dim_);

    scalar_rz_ = alloc<float>(rt, 1);
    scalar_pAp_ = alloc<float>(rt, 1);
    scalar_alpha_ = alloc<float>(rt, 1);
    scalar_beta_ = alloc<float>(rt, 1);
    scalar_rz_old_ = alloc<float>(rt, 1);
    scalar_rr_ = alloc<float>(rt, 1);
    rt.stream.sync();
    SPDLOG_TRACE("[MAS] [factorize_device_buffers] [{:.6f}]",
                 elapsed_seconds(phase_begin));

    // Allocates buffers for CuSparse.
    phase_begin = clock::now();
    setup_cusparse(rt);
    rt.stream.sync();
    SPDLOG_TRACE("[MAS] [factorize_cusparse] [{:.6f}]",
                 elapsed_seconds(phase_begin));
    SPDLOG_TRACE("[MAS] [factorize_total] [{:.6f}]",
                 elapsed_seconds(total_begin));
  }

  void solve(const Eigen::Ref<const Eigen::VectorXd> b,
             Eigen::Ref<Eigen::VectorXd> x) {
    CudaRuntime rt{*default_stream_, default_mem_pool_->as_ref()};

    cu::copy_bytes(rt.stream, ctd::span<const float>(b.data(), dim_),
                   ctd::span<float>(r_->data(), dim_));
    cu::fill_bytes(rt.stream,
                   ctd::span<float>(r_->data() + dim_, permuted_dim_ - dim_),
                   0);
    permute_vector(*r_, *b_, *d_inv_permutation_, A_.view().block_dim, rt);

    // The solver sometimes fails to converge if we use input x as initial
    // value. Maybe the caller does not initialize x properly? Set initial x to
    // zero to work around this issue for now.
    cu::fill_bytes(rt.stream, *x_, 0);

    pcg_solve(rt);

    permute_vector(*x_, *r_, *d_permutation_, A_.view().block_dim, rt);

    cu::copy_bytes(rt.stream, ctd::span<const float>(r_->data(), dim_),
                   ctd::span<float>(x.data(), dim_));
    rt.stream.sync();
  }

  void apply_preconditioner(ctd::span<const float> r, ctd::span<float> z,
                            CudaRuntime rt) {
    mas_precond_.apply(r, z, rt);
    return;
  }

  /// https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf
  void pcg_solve(CudaRuntime rt) {
    // Compute initial residual r = b-Ax.
    spmv(*x_, *r_, rt);
    axpby(1.0, nullptr, -1.0, nullptr, *b_, *r_, rt);

    // Compute z = M^-1 r.
    apply_preconditioner(*r_, *z_, rt);
    // Initial search direction p = z;
    cu::copy_bytes(rt.stream, *z_, *p_);

    // Compute rz = r^T M^-1 r.
    inner_product(*r_, *z_, *scalar_rz_, rt);
    const float rz0 = device2host(scalar_rz_->data(), rt);
    if (ctd::isnan(rz0) || !ctd::isfinite(rz0)) {
      throw std::runtime_error("[MAS] Invalid initial residual.");
    }

    // Compute rr = r^T r.
    float rr0 = 0.0;
    if (!use_preconditioned_residual_norm_) {
      inner_product(*r_, *r_, *scalar_rr_, rt);
      rr0 = device2host(scalar_rr_->data(), rt);
    }

    // debug logging
    auto iter_window_begin = clock::now();
    for (int k = 1; k <= max_iter_; ++k) {
      // Compute Ap = A p.
      spmv(*p_, *Ap_, rt);
      // Compute pAp = p^T * A * p.
      inner_product(*p_, *Ap_, *scalar_pAp_, rt);
      // Compute alpha = (r M^-1 r) / (p^T A p).
      scalar_division(*scalar_rz_, *scalar_pAp_, *scalar_alpha_, rt);
      // Compute x = x + alpha A p.
      axpby(1.0, scalar_alpha_->data(), 1.0, nullptr, *p_, *x_, rt);

      // Compute residual b-Ax directly.
      if (k % true_residual_period_ == 0) {
        spmv(*x_, *r_, rt);
        axpby(1.0, nullptr, -1.0, nullptr, *b_, *r_, rt);
      }
      // Compute residual update using r' = r - alpha A p.
      // This saves one spmv but accumulates floating point error overtime.
      else {
        axpby(-1.0, scalar_alpha_->data(), 1.0, nullptr, *Ap_, *r_, rt);
      }

      // Compute z = M^-1 r.
      apply_preconditioner(*r_, *z_, rt);

      // Compute rz = r M^-1 r.
      cu::copy_bytes(rt.stream, *scalar_rz_, *scalar_rz_old_);
      inner_product(*r_, *z_, *scalar_rz_, rt);

      iterations_ = k;
      bool converged = false;

      // Check convergence every 10 iterations.
      if (k % 10 == 0) {
        if (use_preconditioned_residual_norm_) {
          float rz_new = device2host(scalar_rz_->data(), rt);
          residual_norm_ = ctd::sqrt(rz_new);
          if (rz_new <= rel_tol_ * rel_tol_ * rz0 ||
              rz_new <= abs_tol_ * abs_tol_) {
            status_ = (rz_new <= abs_tol_ * abs_tol_)
                          ? MASSolverStatus::ReachAbsoluteTolerance
                          : MASSolverStatus::ReachRelativeTolerance;
            converged = true;
          }
        } else {
          inner_product(*r_, *r_, *scalar_rr_, rt);
          float rr = device2host(scalar_rr_->data(), rt);
          residual_norm_ = ctd::sqrt(rr);
          if (rr <= rel_tol_ * rel_tol_ * rr0 || rr <= abs_tol_ * abs_tol_) {
            status_ = (rr <= abs_tol_ * abs_tol_)
                          ? MASSolverStatus::ReachAbsoluteTolerance
                          : MASSolverStatus::ReachRelativeTolerance;
            converged = true;
          }
        }
      }

      // debug logging
      if (k % 100 == 0) {
        rt.stream.sync();
        SPDLOG_TRACE("[MAS] [pcg_loop] [{:.6f}] [iter={}] [residual={:.6e}]",
                     elapsed_seconds(iter_window_begin), k, residual_norm_);
        iter_window_begin = clock::now();
      }

      if (converged) {
        break;
      }

      // Compute beta = rz / rz_old.
      scalar_division(*scalar_rz_, *scalar_rz_old_, *scalar_beta_, rt);
      // Compute direction update p' = M^-1 r + beta p.
      axpby(1.0, nullptr, 1.0, scalar_beta_->data(), *z_, *p_, rt);
    }

    if (iterations_ == max_iter_) {
      status_ = MASSolverStatus::ReachMaxIterations;
    }

    SPDLOG_TRACE("[MAS] [pcg_loop] [{:.6f}] [iter={}] [residual={:.6e}]",
                 elapsed_seconds(iter_window_begin), iterations_,
                 residual_norm_);
  }
};

}  // namespace silk::cuda::solver
