#include <cuda_runtime.h>

#include "cloth_solver_kernels.cuh"
#include "solver/gpu/svd.cuh"

namespace silk::gpu {

/**
 * @brief Column-major matrix multiply C = A * B with optional transposes.
 *
 * Dimensions (logical, after applying transpose flags):
 *   - C is I x J
 *   - A is I x K
 *   - B is K x J
 *
 * Storage assumptions:
 *   - transpose_A == false: A is a I x K matrix. Column major.
 *   - transpose_A == true:  A is a K x I matrix. Column major.
 *   - transpose_B == false: B is a K x J matrix. Column major.
 *   - transpose_B == true:  B is a J x K matrix. Column major.
 *
 * Example:
 *   - mat_mul<6,1,9>(J, x, y): compute y = J * x
 *       J: 6 x 9
 *       x: 9 x 1
 *       y: 6 x 1
 *
 *   - mat_mul<9,1,6,true,false>(J, x, y): compute y = J^T * x
 *       J: 6 x 9, also column major.
 *       x: 6 x 1
 *       y: 9 x 1
 */
template <int I, int J, int K, bool transpose_A = false,
          bool transpose_B = false>
__device__ void mat_mul(const float* A, const float* B, float* C) {
#pragma unroll
  for (int i = 0; i < I; ++i) {
#pragma unroll
    for (int j = 0; j < J; ++j) {
      float sum = 0.0f;
#pragma unroll
      for (int k = 0; k < K; ++k) {
        float a;
        if constexpr (transpose_A) {
          a = A[k + i * K];
        } else {
          a = A[i + k * I];
        }

        float b;
        if constexpr (transpose_B) {
          b = B[j + k * J];
        } else {
          b = B[k + j * K];
        }
        sum += a * b;
      }
      C[j * I + i] = sum;
    }
  }
}

/**
 * @brief GPU kernel for computing elastic RHS contributions per face
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
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= ops_num) {
    return;
  }

  // gather vertices position
  int v_idx0 = d_F[tid * 3 + 0];
  int v_idx1 = d_F[tid * 3 + 1];
  int v_idx2 = d_F[tid * 3 + 2];
  int offset0 = v_idx0 * 3;
  int offset1 = v_idx1 * 3;
  int offset2 = v_idx2 * 3;

  float buffer[9];
  buffer[0] = d_state[offset0 + 0];
  buffer[1] = d_state[offset0 + 1];
  buffer[2] = d_state[offset0 + 2];
  buffer[3] = d_state[offset1 + 0];
  buffer[4] = d_state[offset1 + 1];
  buffer[5] = d_state[offset1 + 2];
  buffer[6] = d_state[offset2 + 0];
  buffer[7] = d_state[offset2 + 1];
  buffer[8] = d_state[offset2 + 2];

  // compute 3x2 deformation gradient matrix D
  float D[6];
  const float* jac_op = d_jacobian_ops + tid * 54;
  mat_mul<6, 1, 9>(jac_op, buffer, D);

  // SVD decomposition D = USV^T
  float U[6];
  float V[4];
  float S[2];
  svd32(
      // input D
      D[0], D[1], D[2], D[3], D[4], D[5],
      // output U
      U[0], U[1], U[2], U[3], U[4], U[5],
      // output sigma
      S[0], S[1],
      // output V
      V[0], V[1], V[2], V[3]);

  // compute T = U S' V^t where S is clamped between 0.9 and 1.1
  S[0] = max(0.9f, min(1.1f, S[0]));
  S[1] = max(0.9f, min(1.1f, S[1]));
  float SVT[4];
  SVT[0] = S[0] * V[0];
  SVT[1] = S[1] * V[2];
  SVT[2] = S[0] * V[1];
  SVT[3] = S[1] * V[3];
  float T[6];
  mat_mul<3, 2, 2>(U, SVT, T);

  // compute elastic rhs
  mat_mul<9, 1, 6, true, false>(jac_op, T, buffer);
  float weight = elastic_stiffness * d_areas[tid];
#pragma unroll
  for (int i = 0; i < 9; ++i) {
    buffer[i] *= weight;
  }

  // add back to global rhs vector
  atomicAdd(&d_elastic_rhs[offset0 + 0], buffer[0]);
  atomicAdd(&d_elastic_rhs[offset0 + 1], buffer[1]);
  atomicAdd(&d_elastic_rhs[offset0 + 2], buffer[2]);
  atomicAdd(&d_elastic_rhs[offset1 + 0], buffer[3]);
  atomicAdd(&d_elastic_rhs[offset1 + 1], buffer[4]);
  atomicAdd(&d_elastic_rhs[offset1 + 2], buffer[5]);
  atomicAdd(&d_elastic_rhs[offset2 + 0], buffer[6]);
  atomicAdd(&d_elastic_rhs[offset2 + 1], buffer[7]);
  atomicAdd(&d_elastic_rhs[offset2 + 2], buffer[8]);
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
  // cudaError_t err = cudaGetLastError();
  // if (err != cudaSuccess) {
  //   throw std::runtime_error(std::string("CUDA kernel launch failed: ") +
  //                            cudaGetErrorString(err));
  // }
}

void launch_add_vectors_kernel(int n, const float* a, const float* b,
                               float* result, int block_size) {
  int grid_size = (n + block_size - 1) / block_size;

  add_vectors_kernel<<<grid_size, block_size>>>(n, a, b, result);

  // Check for kernel launch errors
  // cudaError_t err = cudaGetLastError();
  // if (err != cudaSuccess) {
  //   throw std::runtime_error(std::string("CUDA kernel launch failed: ") +
  //                            cudaGetErrorString(err));
  // }
}

}  // namespace silk::gpu
