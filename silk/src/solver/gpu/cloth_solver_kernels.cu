#include "cloth_solver_kernels.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

// --- Eigen CUDA Setup ---
#define EIGEN_NO_MALLOC  // Disables all dynamic memory allocation
// #define EIGEN_USE_GPU    // Enables __host__ __device__ annotations

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace silk {
namespace gpu {

// --- Helper Functions ---

/**
 * @brief Device-side clamp function for float values
 */
__device__ __forceinline__ float d_clamp(float val, float lo, float hi) {
  return fmaxf(lo, fminf(val, hi));
}

// --- Type Aliases ---
using Vector2f = Eigen::Matrix<float, 2, 1>;
using Vector3f = Eigen::Matrix<float, 3, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix2f = Eigen::Matrix<float, 2, 2>;
using Matrix3f = Eigen::Matrix<float, 3, 3>;
using Matrix3x2f = Eigen::Matrix<float, 3, 2>;
using Matrix6x9f = Eigen::Matrix<float, 6, 9>;

// ====================================================================
// KERNEL: Elastic RHS Computation
// ====================================================================

/**
 * @brief GPU kernel for computing elastic RHS contributions per face
 *
 * This kernel implements the per-face elastic projection step from the CPU
 * cloth solver's inner loop. Each thread processes one triangle face:
 * 1. Gathers vertex positions from global state
 * 2. Computes deformation gradient via Jacobian operator
 * 3. Performs SVD decomposition and clamps singular values
 * 4. Projects to target configuration (rotation + limited stretch)
 * 5. Accumulates elastic force contribution to global RHS
 *
 * @param ops_num Number of triangle faces to process
 * @param d_F Face connectivity array [ops_num * 3] (vertex indices per face)
 * @param d_state Global vertex positions [state_num] (3 * vnum)
 * @param d_jacobian_ops Per-face 6x9 Jacobian matrices [ops_num * 54]
 * @param d_areas Per-face rest areas [ops_num]
 * @param elastic_stiffness Material stiffness parameter
 * @param d_elastic_rhs Output: accumulated elastic forces [state_num]
 */
__global__ void compute_elastic_rhs_kernel(int ops_num, const int* d_F,
                                           const float* d_state,
                                           const float* d_jacobian_ops,
                                           const float* d_areas,
                                           float elastic_stiffness,
                                           float* d_elastic_rhs) {
  // --- 1. Get this thread's triangle index 'i' ---
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= ops_num) {
    return;  // Out of bounds
  }

  // --- 2. Get vertex indices for this triangle ---
  int v_idx_0 = d_F[i * 3 + 0];
  int v_idx_1 = d_F[i * 3 + 1];
  int v_idx_2 = d_F[i * 3 + 2];

  int offset0 = v_idx_0 * 3;
  int offset1 = v_idx_1 * 3;
  int offset2 = v_idx_2 * 3;

  // --- 3. Assemble local 9D position buffer ---
  Vector9f buffer;
  buffer(0) = d_state[offset0 + 0];
  buffer(1) = d_state[offset0 + 1];
  buffer(2) = d_state[offset0 + 2];
  buffer(3) = d_state[offset1 + 0];
  buffer(4) = d_state[offset1 + 1];
  buffer(5) = d_state[offset1 + 2];
  buffer(6) = d_state[offset2 + 0];
  buffer(7) = d_state[offset2 + 1];
  buffer(8) = d_state[offset2 + 2];

  // --- 4. Compute deformation gradient: D = jac_op * buffer ---
  // Map the jacobian operator as a 6x9 row-major matrix
  Eigen::Map<const Eigen::Matrix<float, 6, 9, Eigen::RowMajor>> jac_op(
      &d_jacobian_ops[i * 54]);

  Vector6f D_vec = jac_op * buffer;
  Eigen::Map<Matrix3x2f> D(D_vec.data());  // Reshape to 3x2

  // --- 5. SVD via eigendecomposition (A = D^T * D) ---
  // For 3x2 matrix D, we compute SVD via D^T*D eigendecomposition
  Matrix2f A = D.transpose() * D;
  Eigen::SelfAdjointEigenSolver<Matrix2f> es(A, Eigen::ComputeEigenvectors);

  Matrix2f V = es.eigenvectors();
  Vector2f S_squared = es.eigenvalues();

  // Ensure non-negative eigenvalues (numerical safety)
  S_squared(0) = fmaxf(S_squared(0), 0.0f);
  S_squared(1) = fmaxf(S_squared(1), 0.0f);

  Vector2f sigma = S_squared.cwiseSqrt();  // Singular values

  // Clamp singular values to limit stretch [0.9, 1.1]
  Vector2f sigma_clamped;
  sigma_clamped(0) = d_clamp(sigma(0), 0.9f, 1.1f);
  sigma_clamped(1) = d_clamp(sigma(1), 0.9f, 1.1f);

  // --- 6. Reconstruct U and target deformation T = U * S_clamped * V^T ---
  constexpr float epsilon = 1e-6f;
  Vector3f u0 = (D * V.col(0)) / (sigma(0) + epsilon);
  Vector3f u1 = (D * V.col(1)) / (sigma(1) + epsilon);
  Vector3f u2 = u0.cross(u1);  // Third column via cross product

  Matrix3f U;
  U.col(0) = u0;
  U.col(1) = u1;
  U.col(2) = u2;

  Matrix3x2f T = U.block<3, 2>(0, 0) * sigma_clamped.asDiagonal() *
                 V.transpose();

  // --- 7. Compute elastic force: buffer = weight * jac_op^T * T ---
  float weight = elastic_stiffness * d_areas[i];
  Eigen::Map<Vector6f> T_vec(T.data());

  buffer = weight * jac_op.transpose() * T_vec;

  // --- 8. Accumulate to global RHS using atomic operations ---
  // Atomic add is required since multiple threads may write to same vertex
  atomicAdd(&d_elastic_rhs[offset0 + 0], buffer(0));
  atomicAdd(&d_elastic_rhs[offset0 + 1], buffer(1));
  atomicAdd(&d_elastic_rhs[offset0 + 2], buffer(2));

  atomicAdd(&d_elastic_rhs[offset1 + 0], buffer(3));
  atomicAdd(&d_elastic_rhs[offset1 + 1], buffer(4));
  atomicAdd(&d_elastic_rhs[offset1 + 2], buffer(5));

  atomicAdd(&d_elastic_rhs[offset2 + 0], buffer(6));
  atomicAdd(&d_elastic_rhs[offset2 + 1], buffer(7));
  atomicAdd(&d_elastic_rhs[offset2 + 2], buffer(8));
}

// ====================================================================
// KERNEL: Vector Addition
// ====================================================================

/**
 * @brief Simple element-wise vector addition kernel
 *
 * Computes: result[i] = a[i] + b[i]
 * Used to combine outer_rhs with elastic_rhs
 *
 * @param n Vector length
 * @param a First input vector
 * @param b Second input vector
 * @param result Output vector
 */
__global__ void add_vectors_kernel(int n, const float* a, const float* b,
                                   float* result) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    result[i] = a[i] + b[i];
  }
}

// ====================================================================
// Host-side Wrapper Functions
// ====================================================================

void launch_compute_elastic_rhs_kernel(int ops_num, const int* d_F,
                                       const float* d_state,
                                       const float* d_jacobian_ops,
                                       const float* d_areas,
                                       float elastic_stiffness,
                                       float* d_elastic_rhs, int block_size) {
  int grid_size = (ops_num + block_size - 1) / block_size;

  compute_elastic_rhs_kernel<<<grid_size, block_size>>>(
      ops_num, d_F, d_state, d_jacobian_ops, d_areas, elastic_stiffness,
      d_elastic_rhs);

  // Check for kernel launch errors
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA kernel launch failed: ") +
                             cudaGetErrorString(err));
  }
}

void launch_add_vectors_kernel(int n, const float* a, const float* b,
                                float* result, int block_size) {
  int grid_size = (n + block_size - 1) / block_size;

  add_vectors_kernel<<<grid_size, block_size>>>(n, a, b, result);

  // Check for kernel launch errors
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA kernel launch failed: ") +
                             cudaGetErrorString(err));
  }
}

}  // namespace gpu
}  // namespace silk
