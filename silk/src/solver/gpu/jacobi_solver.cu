/** @file
 * CUDA implementation of Jacobi iterative solver.
 */

// Prevent NVCC from including x86 intrinsic headers that cause compilation errors
#ifdef __CUDACC__
#define _AMXTILEINTRIN_H_INCLUDED
#define _AMXBF16INTRIN_H_INCLUDED
#define _AMXINT8INTRIN_H_INCLUDED
#define _AMXFP16INTRIN_H_INCLUDED
#endif

#include "jacobi_solver.cuh"

#include <iostream>
#include <vector>
#include <cmath>

namespace silk {
namespace gpu {

// -----------------------------------------------------------------
// CUDA Kernels
// -----------------------------------------------------------------

/**
 * @brief Jacobi iteration kernel using separate diagonal and off-diagonal storage.
 *
 * Computes: x_new[i] = (b[i] - sum(R[i,j] * x_old[j])) / D[i]
 *
 * @param r_row_ptr CSR row pointers for R matrix
 * @param r_col_idx CSR column indices for R matrix
 * @param r_values CSR values for R matrix (off-diagonal only)
 * @param diag Diagonal values D[i]
 * @param b Right-hand side vector
 * @param x_old Previous solution
 * @param x_new Output: new solution (output)
 * @param n Matrix size
 */
__global__ void jacobi_iteration_kernel(const int* r_row_ptr,
                                        const int* r_col_idx,
                                        const float* r_values,
                                        const float* diag, const float* b,
                                        const float* x_old, float* x_new,
                                        int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < n) {
    // Compute R*x_old (off-diagonal contribution)
    float sum = 0.0f;
    int row_start = r_row_ptr[i];
    int row_end = r_row_ptr[i + 1];

    for (int j = row_start; j < row_end; ++j) {
      sum += r_values[j] * x_old[r_col_idx[j]];
    }

    // Jacobi update: x_new[i] = (b[i] - sum) / D[i]
    float diag_val = diag[i];
    if (diag_val != 0.0f) {
      x_new[i] = (b[i] - sum) / diag_val;
    } else {
      x_new[i] = 0.0f;  // Handle zero diagonal (should not occur in SPD)
    }
  }
}

/**
 * @brief Compute residual: r = Ax - b = (D + R)*x - b
 *
 * @param r_row_ptr CSR row pointers for R matrix
 * @param r_col_idx CSR column indices for R matrix
 * @param r_values CSR values for R matrix
 * @param diag Diagonal values D[i]
 * @param x Current solution
 * @param b Right-hand side
 * @param residual Output: residual vector
 * @param n Matrix size
 */
__global__ void compute_residual_kernel(const int* r_row_ptr,
                                        const int* r_col_idx,
                                        const float* r_values,
                                        const float* diag, const float* x,
                                        const float* b, float* residual, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < n) {
    float Ax_i = 0.0f;

    // 1. Compute R*x (off-diagonal part)
    int row_start = r_row_ptr[i];
    int row_end = r_row_ptr[i + 1];
    for (int j = row_start; j < row_end; ++j) {
      Ax_i += r_values[j] * x[r_col_idx[j]];
    }

    // 2. Add D*x (diagonal part)
    Ax_i += diag[i] * x[i];

    // 3. Compute residual: r = Ax - b
    residual[i] = Ax_i - b[i];
  }
}

// -----------------------------------------------------------------
// GpuJacobiSolver Implementation
// -----------------------------------------------------------------

GpuJacobiSolver::~GpuJacobiSolver() { cleanup(); }

GpuJacobiSolver::GpuJacobiSolver(GpuJacobiSolver&& other) noexcept
    : n_(other.n_),
      nnz_r_(other.nnz_r_),
      d_r_row_ptr_(other.d_r_row_ptr_),
      d_r_col_idx_(other.d_r_col_idx_),
      d_r_values_(other.d_r_values_),
      d_diag_(other.d_diag_),
      d_b_(other.d_b_),
      d_x_old_(other.d_x_old_),
      d_x_new_(other.d_x_new_),
      d_residual_(other.d_residual_) {
  // Nullify other's pointers
  other.n_ = 0;
  other.nnz_r_ = 0;
  other.d_r_row_ptr_ = nullptr;
  other.d_r_col_idx_ = nullptr;
  other.d_r_values_ = nullptr;
  other.d_diag_ = nullptr;
  other.d_b_ = nullptr;
  other.d_x_old_ = nullptr;
  other.d_x_new_ = nullptr;
  other.d_residual_ = nullptr;
}

GpuJacobiSolver& GpuJacobiSolver::operator=(GpuJacobiSolver&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  cleanup();

  n_ = other.n_;
  nnz_r_ = other.nnz_r_;
  d_r_row_ptr_ = other.d_r_row_ptr_;
  d_r_col_idx_ = other.d_r_col_idx_;
  d_r_values_ = other.d_r_values_;
  d_diag_ = other.d_diag_;
  d_b_ = other.d_b_;
  d_x_old_ = other.d_x_old_;
  d_x_new_ = other.d_x_new_;
  d_residual_ = other.d_residual_;

  other.n_ = 0;
  other.nnz_r_ = 0;
  other.d_r_row_ptr_ = nullptr;
  other.d_r_col_idx_ = nullptr;
  other.d_r_values_ = nullptr;
  other.d_diag_ = nullptr;
  other.d_b_ = nullptr;
  other.d_x_old_ = nullptr;
  other.d_x_new_ = nullptr;
  other.d_residual_ = nullptr;

  return *this;
}

bool GpuJacobiSolver::setup(
    const Eigen::SparseMatrix<float, Eigen::RowMajor>& A,
    Eigen::VectorXf& D_host) {
  cleanup();  // Clean up any previous allocation

  n_ = A.rows();
  if (n_ == 0) {
    std::cerr << "Error: Empty matrix passed to GpuJacobiSolver::setup\n";
    return false;
  }

  // Split A into D (diagonal) and R (off-diagonal)
  std::vector<Eigen::Triplet<float>> r_triplets;
  D_host.resize(n_);

  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<float, Eigen::RowMajor>::InnerIterator it(A, k);
         it; ++it) {
      if (it.row() == it.col()) {
        D_host(it.row()) = it.value();
      } else {
        r_triplets.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }

  // Build R matrix
  Eigen::SparseMatrix<float, Eigen::RowMajor> R_host(n_, n_);
  R_host.setFromTriplets(r_triplets.begin(), r_triplets.end());
  R_host.makeCompressed();

  nnz_r_ = R_host.nonZeros();

  // Allocate GPU memory
  CUDA_CHECK(cudaMalloc(&d_r_row_ptr_, (n_ + 1) * sizeof(int)));
  CUDA_CHECK(cudaMalloc(&d_r_col_idx_, nnz_r_ * sizeof(int)));
  CUDA_CHECK(cudaMalloc(&d_r_values_, nnz_r_ * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_diag_, n_ * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_b_, n_ * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_x_old_, n_ * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_x_new_, n_ * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_residual_, n_ * sizeof(float)));

  // Upload R matrix (one-time upload)
  CUDA_CHECK(cudaMemcpy(d_r_row_ptr_, R_host.outerIndexPtr(),
                        (n_ + 1) * sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_r_col_idx_, R_host.innerIndexPtr(),
                        nnz_r_ * sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_r_values_, R_host.valuePtr(),
                        nnz_r_ * sizeof(float), cudaMemcpyHostToDevice));

  return true;
}

bool GpuJacobiSolver::solve(const Eigen::VectorXf& D_host,
                            const Eigen::VectorXf& b_host,
                            Eigen::VectorXf& x_host, int max_iter, float tol) {
  if (!is_initialized()) {
    std::cerr << "Error: GpuJacobiSolver not initialized. Call setup() first.\n";
    return false;
  }

  int n = b_host.size();
  if (n != n_) {
    std::cerr << "Error: Solver setup for n=" << n_ << " but called with n=" << n
              << std::endl;
    return false;
  }

  // Upload D and b (per-frame upload - only what changed)
  CUDA_CHECK(
      cudaMemcpy(d_diag_, D_host.data(), n * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(
      cudaMemcpy(d_b_, b_host.data(), n * sizeof(float), cudaMemcpyHostToDevice));

  // Initialize x = 0
  CUDA_CHECK(cudaMemset(d_x_old_, 0, n * sizeof(float)));

  // Kernel launch parameters
  int block_size = 256;
  int grid_size = (n + block_size - 1) / block_size;

  // Host buffer for convergence checking
  std::vector<float> h_residual(n);
  bool converged = false;

  // Jacobi iteration loop
  for (int iter = 0; iter < max_iter; ++iter) {
    // 1. Perform one Jacobi iteration: x_new = f(x_old)
    jacobi_iteration_kernel<<<grid_size, block_size>>>(
        d_r_row_ptr_, d_r_col_idx_, d_r_values_, d_diag_, d_b_, d_x_old_,
        d_x_new_, n);
    CUDA_CHECK(cudaGetLastError());

    // 2. Swap buffers: x_old now contains the latest solution
    swap_solution_buffers();

    // 3. Check convergence every 10 iterations
    if (iter % 10 == 0 || iter == max_iter - 1) {
      // Compute residual: r = A*x_old - b
      compute_residual_kernel<<<grid_size, block_size>>>(
          d_r_row_ptr_, d_r_col_idx_, d_r_values_, d_diag_, d_x_old_, d_b_,
          d_residual_, n);
      CUDA_CHECK(cudaGetLastError());

      // Copy residual to host
      CUDA_CHECK(cudaMemcpy(h_residual.data(), d_residual_, n * sizeof(float),
                            cudaMemcpyDeviceToHost));

      // Compute norm
      float residual_norm = 0.0f;
      for (int i = 0; i < n; ++i) {
        residual_norm += h_residual[i] * h_residual[i];
      }
      residual_norm = std::sqrt(residual_norm);

      // Check convergence
      if (residual_norm < tol) {
        converged = true;
        break;
      }
    }
  }

  // Copy solution back to host (solution is in d_x_old after swap)
  CUDA_CHECK(
      cudaMemcpy(x_host.data(), d_x_old_, n * sizeof(float), cudaMemcpyDeviceToHost));

  return true;
}

void GpuJacobiSolver::cleanup() {
  CUDA_CHECK_VOID(cudaFree(d_r_row_ptr_));
  CUDA_CHECK_VOID(cudaFree(d_r_col_idx_));
  CUDA_CHECK_VOID(cudaFree(d_r_values_));
  CUDA_CHECK_VOID(cudaFree(d_diag_));
  CUDA_CHECK_VOID(cudaFree(d_b_));
  CUDA_CHECK_VOID(cudaFree(d_x_old_));
  CUDA_CHECK_VOID(cudaFree(d_x_new_));
  CUDA_CHECK_VOID(cudaFree(d_residual_));

  d_r_row_ptr_ = nullptr;
  d_r_col_idx_ = nullptr;
  d_r_values_ = nullptr;
  d_diag_ = nullptr;
  d_b_ = nullptr;
  d_x_old_ = nullptr;
  d_x_new_ = nullptr;
  d_residual_ = nullptr;

  n_ = 0;
  nnz_r_ = 0;
}

void GpuJacobiSolver::swap_solution_buffers() {
  float* temp = d_x_old_;
  d_x_old_ = d_x_new_;
  d_x_new_ = temp;
}

}  // namespace gpu
}  // namespace silk
