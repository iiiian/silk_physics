/** @file
 * CUDA-accelerated Jacobi iterative solver for sparse linear systems.
 *
 * Provides GPU-accelerated solving of Ax = b using the Jacobi iteration method.
 * The matrix A is split into diagonal (D) and off-diagonal (R) components:
 * A = D + R, where D is stored as a dense vector and R as a sparse matrix.
 *
 * Key optimizations:
 * - One-time upload of static matrix structure (R)
 * - Per-frame upload of only changing data (D, b)
 * - Device memory persistence across solver calls
 * - Efficient CSR sparse matrix representation
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cuda_runtime.h>

namespace silk {
namespace gpu {

// CUDA error checking macro
#define CUDA_CHECK(call)                                                     \
  do {                                                                       \
    cudaError_t err = call;                                                  \
    if (err != cudaSuccess) {                                                \
      std::cerr << "CUDA Error: " << cudaGetErrorString(err) << " at "      \
                << __FILE__ << ":" << __LINE__ << std::endl;                 \
      return false;                                                          \
    }                                                                        \
  } while (0)

#define CUDA_CHECK_VOID(call)                                                \
  do {                                                                       \
    cudaError_t err = call;                                                  \
    if (err != cudaSuccess) {                                                \
      std::cerr << "CUDA Error: " << cudaGetErrorString(err) << " at "      \
                << __FILE__ << ":" << __LINE__ << std::endl;                 \
    }                                                                        \
  } while (0)

/**
 * @brief GPU Jacobi solver state managing device memory and solver context.
 *
 * This class manages persistent GPU memory for the Jacobi solver, allowing
 * efficient reuse across multiple solver calls. The matrix A is split into:
 * - R: Off-diagonal elements (stored as CSR sparse matrix on GPU)
 * - D: Diagonal elements (stored as dense vector on GPU)
 *
 * Memory layout:
 * - Static (uploaded once): R matrix (row_ptr, col_idx, values)
 * - Dynamic (per-frame): D vector, b vector
 * - Temporary: x_old, x_new, residual
 */
class GpuJacobiSolver {
 public:
  GpuJacobiSolver() = default;
  ~GpuJacobiSolver();

  // Non-copyable due to device pointers
  GpuJacobiSolver(const GpuJacobiSolver&) = delete;
  GpuJacobiSolver& operator=(const GpuJacobiSolver&) = delete;

  // Movable
  GpuJacobiSolver(GpuJacobiSolver&& other) noexcept;
  GpuJacobiSolver& operator=(GpuJacobiSolver&& other) noexcept;

  /**
   * @brief Initialize solver with matrix structure.
   *
   * Splits input matrix A into D (diagonal) and R (off-diagonal),
   * allocates all GPU memory, and uploads the static R matrix.
   *
   * @param A Input sparse matrix (will be split into D + R)
   * @param D_host Output diagonal vector (extracted from A)
   * @return true on success, false on CUDA error
   */
  bool setup(const Eigen::SparseMatrix<float, Eigen::RowMajor>& A,
             Eigen::VectorXf& D_host);

  /**
   * @brief Solve Ax = b using Jacobi iteration.
   *
   * Assumes setup() has been called. Only uploads D and b to GPU,
   * reusing the previously uploaded R matrix structure.
   *
   * @param D_host Diagonal vector (may have been modified since setup)
   * @param b_host Right-hand side vector
   * @param x_host Output solution vector
   * @param max_iter Maximum number of iterations
   * @param tol Residual norm tolerance for convergence
   * @return true if converged or max iterations reached, false on error
   */
  bool solve(const Eigen::VectorXf& D_host, const Eigen::VectorXf& b_host,
             Eigen::VectorXf& x_host, int max_iter, float tol);

  /**
   * @brief Free all GPU memory.
   */
  void cleanup();

  /**
   * @brief Check if solver has been initialized.
   */
  bool is_initialized() const { return n_ > 0; }

  /**
   * @brief Get matrix size.
   */
  int size() const { return n_; }

 private:
  // Matrix dimensions
  int n_ = 0;        // Matrix size
  int nnz_r_ = 0;    // Number of non-zeros in R

  // Device pointers - R matrix (static, uploaded once)
  int* d_r_row_ptr_ = nullptr;
  int* d_r_col_idx_ = nullptr;
  float* d_r_values_ = nullptr;

  // Device pointers - D, b (dynamic, uploaded per-frame)
  float* d_diag_ = nullptr;
  float* d_b_ = nullptr;

  // Device pointers - solver state (temporary)
  float* d_x_old_ = nullptr;
  float* d_x_new_ = nullptr;
  float* d_residual_ = nullptr;

  /**
   * @brief Swap x_old and x_new pointers.
   */
  void swap_solution_buffers();
};

}  // namespace gpu
}  // namespace silk
