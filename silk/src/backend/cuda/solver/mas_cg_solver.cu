#include "backend/cuda/solver/mas_cg_solver.cuh"

#include <cuda_runtime.h>

#include <chrono>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/span>
#include <nvtx3/nvtx3.hpp>
#include <utility>

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cugraph_wrapper.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/solver/inner_product.cuh"
#include "backend/cuda/solver/mas_preconditioner.cuh"
#include "common/logger.hpp"

namespace silk::cuda {

namespace {

using clock = std::chrono::steady_clock;

[[maybe_unused]] float elapsed(const std::chrono::time_point<clock> &begin) {
  return std::chrono::duration<float>(clock::now() - begin).count();
}

/// Device scalar devision num / denom.
/// Defaults to zero when denom is small because beta should be zero after
/// arriving at the solution.
void scalar_division(ctd::span<const float> num, ctd::span<const float> denom,
                     ctd::span<float> out, CudaRuntime rt) {
  auto op = [num, denom, out] __device__(int) {
    out[0] = (ctd::abs(denom[0]) < 1e-20) ? 0.0 : (num[0] / denom[0]);
  };
  cub::DeviceFor::Bulk(1, op, rt.stream.get());
}

__global__ void compute_residual_kernel(ctd::span<const float> b,
                                        ctd::span<float> r) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= r.size()) {
    return;
  }
  r[tid] = b[tid] - r[tid];
}

__global__ void update_x_kernel(ctd::span<const float> p, const float *d_alpha,
                                ctd::span<float> x) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= x.size()) {
    return;
  }
  float alpha = *d_alpha;
  x[tid] = alpha * p[tid] + x[tid];
}

__global__ void update_r_kernel(ctd::span<const float> Ap, const float *d_alpha,
                                ctd::span<float> r) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= r.size()) {
    return;
  }
  float alpha = *d_alpha;
  r[tid] = -alpha * Ap[tid] + r[tid];
}

__global__ void update_p_kernel(ctd::span<const float> z, const float *d_beta,
                                ctd::span<float> p) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= p.size()) {
    return;
  }
  float beta = *d_beta;
  p[tid] = z[tid] + beta * p[tid];
}

__global__ void add_diag_kernel(ctd::span<const float> diag,
                                ctd::span<const float> x, ctd::span<float> y) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= diag.size()) {
    return;
  }
  y[idx] += diag[idx] * x[idx];
}

__global__ void check_convergence(cudaGraphConditionalHandle cond_handle,
                                  const float *rr, int *iter, int *status,
                                  float rr0, float rel_tol, float abs_tol,
                                  int max_iter) {
  *iter += 1;
  if (*iter >= max_iter) {
    *status = static_cast<int>(MASCGSolver::Status::ReachMaxIter);
    cudaGraphSetConditional(cond_handle, 0);
    return;
  }

  if (*rr <= abs_tol * abs_tol) {
    *status = static_cast<int>(MASCGSolver::Status::ReachAbsTol);
    cudaGraphSetConditional(cond_handle, 0);
  } else if (*rr <= rel_tol * rel_tol * rr0) {
    *status = static_cast<int>(MASCGSolver::Status::ReachRelTol);
    cudaGraphSetConditional(cond_handle, 0);
  } else {
    cudaGraphSetConditional(cond_handle, 1);
  }
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

  int grid_num = div_round_up(diag_.size(), 128);
  add_diag_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(diag_, x, y);
}

void MASCGSolver::factorize(DynamicBSRView A, ctd::span<const int> part_offset,
                            CudaRuntime rt) {
  assert(A.mat.dim != 0);
  assert(A.diag.size() == A.mat.dim * A.mat.block_dim);

  auto total_begin = clock::now();

  // Initialize MAS.
  auto phase_begin = clock::now();
  mas_precond_.factorize(A, part_offset, rt);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_mas] [{:.6f}]", elapsed(phase_begin));

  phase_begin = clock::now();

  fine_dim_ = A.mat.dim * A.mat.block_dim;
  diag_ = A.diag;
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
  scalar_iter_ = alloc<int>(rt, 1);
  scalar_status_ = alloc<int>(rt, 1);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_device_buffers] [{:.6f}]", elapsed(phase_begin));

  // Allocates buffers for CuSparse.
  phase_begin = clock::now();
  setup_cusparse(A.mat, rt);
  rt.stream.sync();

  SPDLOG_TRACE("[factorize_cusparse] [{:.6f}]", elapsed(phase_begin));
  SPDLOG_TRACE("[factorize_total] [{:.6f}]", elapsed(total_begin));
}

/// https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf
MASCGSolver::Status MASCGSolver::solve(DynamicBSRView A,
                                       ctd::span<const float> b,
                                       ctd::span<float> x, CudaRuntime rt) {
  static const nvtx3::registered_string name{"MASCGSolver::solve"};
  nvtx3::scoped_range range{name};

  assert(b.size() == fine_dim_);
  assert(x.size() == fine_dim_);

  diag_ = A.diag;

  // Compute initial residual r = b-Ax.
  spmv(x, *r_, rt);
  int grid_num = div_round_up(b.size(), 128);
  compute_residual_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(b, *r_);

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
  inner_product(*r_, *r_, *scalar_rr_, rt);
  float rr0 = scalar_load(scalar_rr_->data(), rt);
  residual_norm_ = std::sqrt(rr0);

  cu::fill_bytes(rt.stream, *scalar_iter_, 0);
  auto solve_begin = clock::now();

  // Update cuda graph.
  CudaGraph updated_graph;
  // Setup while node.
  cudaGraphConditionalHandle cond_handle;
  check_cuda(cudaGraphConditionalHandleCreate_v2(
      &cond_handle, updated_graph.raw(), nullptr, 1,
      cudaGraphCondAssignDefault));
  cudaGraphNode_t while_node;
  cudaGraphNodeParams node_params{};
  node_params.type = cudaGraphNodeTypeConditional;
  node_params.conditional = {.handle = cond_handle,
                             .type = cudaGraphCondTypeWhile,
                             .size = 1,
                             .phGraph_out = nullptr,
                             .ctx = nullptr};
  check_cuda(cudaGraphAddNode(&while_node, updated_graph.raw(), nullptr,
                              nullptr, 0, &node_params));

  cudaGraph_t body_graph = node_params.conditional.phGraph_out[0];
  check_cuda(cudaStreamBeginCaptureToGraph(rt.stream.get(), body_graph, nullptr,
                                           nullptr, 0,
                                           cudaStreamCaptureModeThreadLocal));

  // -------------------------------------------
  // Start capture PCG logic
  // -------------------------------------------

  // Compute Ap = A p.
  spmv(*p_, *Ap_, rt);
  // Compute pAp = p^T * A * p.
  inner_product(*p_, *Ap_, *scalar_pAp_, rt);
  // Compute alpha = (r M^-1 r) / (p^T A p).
  scalar_division(*scalar_rz_, *scalar_pAp_, *scalar_alpha_, rt);
  // Compute x = x + alpha p.
  grid_num = div_round_up(x.size(), 128);
  update_x_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      *p_, scalar_alpha_->data(), x);

  // Compute residual update using r' = r - alpha A p.
  grid_num = div_round_up(r_->size(), 128);
  update_r_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      *Ap_, scalar_alpha_->data(), *r_);

  // Compute z = M^-1 r.
  mas_precond_.apply(*r_, *z_, rt);

  // Compute rz = r M^-1 r.
  check_cuda(cudaMemcpyAsync(scalar_rz_old_->data(), scalar_rz_->data(),
                             sizeof(float), cudaMemcpyDeviceToDevice,
                             rt.stream.get()));
  inner_product(*r_, *z_, *scalar_rz_, rt);

  // Check convergence.
  inner_product(*r_, *r_, *scalar_rr_, rt);
  check_convergence<<<1, 1, 0, rt.stream.get()>>>(
      cond_handle, scalar_rr_->data(), scalar_iter_->data(),
      scalar_status_->data(), rr0, rel_tol, abs_tol, max_iter);

  // Compute beta = rz / rz_old.
  scalar_division(*scalar_rz_, *scalar_rz_old_, *scalar_beta_, rt);
  // Compute direction update p' = M^-1 r + beta p.
  grid_num = div_round_up(p_->size(), 128);
  update_p_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      *z_, scalar_beta_->data(), *p_);

  // -------------------------------------------
  // Stop capture PCG logic
  // -------------------------------------------

  cudaGraph_t captured_graph;
  check_cuda(cudaStreamEndCapture(rt.stream.get(), &captured_graph));
  assert(captured_graph == body_graph);

  solve_graph_.update(std::move(updated_graph));
  solve_graph_.launch(rt);

  // Transfer solver status to host.
  float rr;
  int status;
  cudaMemcpyAsync(&iterations_, scalar_iter_->data(), sizeof(iterations_),
                  cudaMemcpyDeviceToHost, rt.stream.get());
  cudaMemcpyAsync(&rr, scalar_rr_->data(), sizeof(rr), cudaMemcpyDeviceToHost,
                  rt.stream.get());
  cudaMemcpyAsync(&status, scalar_status_->data(), sizeof(status),
                  cudaMemcpyDeviceToHost, rt.stream.get());
  rt.stream.sync();
  residual_norm_ = std::sqrt(rr);

  SPDLOG_DEBUG("[pcg_loop] [{:.6f}] [iter={}] [residual={:.6e}]",
               elapsed(solve_begin), iterations_, residual_norm_);

  return static_cast<Status>(status);
}

}  // namespace silk::cuda
