#include <cuda_runtime.h>

#include <cassert>

#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/solver/cloth_solver_kernel.hpp"
#include "backend/cuda/solver/svd.cuh"

namespace silk::cuda {

__global__ void vector_add_kernel(int num, const float* d_a, const float* d_b,
                                  float* d_c) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num) {
    d_c[tid] = d_a[tid] + d_b[tid];
  }
}

void vector_add(int num, const float* d_a, const float* d_b, float* d_c) {
  assert(num != 0);
  assert(d_a && d_b && d_c);

  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     vector_add_kernel, 0, 0);
  int grid_size = (num + block_size - 1) / block_size;

  vector_add_kernel<<<grid_size, block_size>>>(num, d_a, d_b, d_c);
}

__global__ void compute_outer_rhs_kernel(
    int state_num, float dt, float acc_x, float acc_y, float acc_z,
    const float* d_mass, const float* d_state, const float* d_state_velocity,
    const float* d_barrier_rhs, float* d_rhs) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (3 * tid < state_num) {
    int o1 = 3 * tid;
    int o2 = 3 * tid + 1;
    int o3 = 3 * tid + 2;
    d_rhs[o1] += (d_mass[o1] / (dt * dt)) * d_state[o1] +
                 (d_mass[o1] / dt) * d_state_velocity[o1] + d_mass[o1] * acc_x +
                 d_barrier_rhs[o1];
    d_rhs[o2] += (d_mass[o2] / (dt * dt)) * d_state[o2] +
                 (d_mass[o2] / dt) * d_state_velocity[o2] + d_mass[o2] * acc_y +
                 d_barrier_rhs[o2];
    d_rhs[o3] += (d_mass[o3] / (dt * dt)) * d_state[o3] +
                 (d_mass[o3] / dt) * d_state_velocity[o3] + d_mass[o3] * acc_z +
                 d_barrier_rhs[o3];
  }
}

void compute_outer_rhs(int state_num, float dt, float acc_x, float acc_y,
                       float acc_z, const float* d_mass, const float* d_state,
                       const float* d_state_velocity,
                       const float* d_barrier_rhs, float* d_rhs) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     compute_outer_rhs_kernel, 0, 0);
  int grid_size = (state_num / 3 + block_size - 1) / block_size;

  compute_outer_rhs_kernel<<<grid_size, block_size>>>(
      state_num, dt, acc_x, acc_y, acc_z, d_mass, d_state, d_state_velocity,
      d_barrier_rhs, d_rhs);
}

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

__global__ void compute_elastic_rhs_kernel(int face_num,
                                           float elastic_stiffness,
                                           const int* d_F, const float* d_state,
                                           const float* d_jacobian_ops,
                                           const float* d_areas, float* d_rhs) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= face_num) {
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

  // SVD decomposition D = U S V^T via svd32 (backed by the 3x3 ref SVD).
  float U[6];
  float V[4];
  float S[2];
  svd32(
      // input D encoded as [d11,d21,d31,d12,d22,d32]
      D[0], D[1], D[2], D[3], D[4], D[5],
      // output U (3x2, column-major)
      U[0], U[1], U[2], U[3], U[4], U[5],
      // output singular values
      S[0], S[1],
      // output V (2x2, column-major)
      V[0], V[1], V[2], V[3]);

  // Clamp singular values to [0.9, 1.1] as in the CPU solver.
  S[0] = max(0.9f, min(1.1f, S[0]));
  S[1] = max(0.9f, min(1.1f, S[1]));

  // Compute T = U S' V^T where S' is clamped.
  float SVT[4];
  // Σ' V^T in column-major:
  //   [s1 * v11, s2 * v12,
  //    s1 * v21, s2 * v22]
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
  atomicAdd(&d_rhs[offset0 + 0], buffer[0]);
  atomicAdd(&d_rhs[offset0 + 1], buffer[1]);
  atomicAdd(&d_rhs[offset0 + 2], buffer[2]);
  atomicAdd(&d_rhs[offset1 + 0], buffer[3]);
  atomicAdd(&d_rhs[offset1 + 1], buffer[4]);
  atomicAdd(&d_rhs[offset1 + 2], buffer[5]);
  atomicAdd(&d_rhs[offset2 + 0], buffer[6]);
  atomicAdd(&d_rhs[offset2 + 1], buffer[7]);
  atomicAdd(&d_rhs[offset2 + 2], buffer[8]);
}

void compute_elastic_rhs(int face_num, float elastic_stiffness, const int* d_F,
                         const float* d_state, const float* d_jacobian_ops,
                         const float* d_areas, float* d_rhs) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     compute_elastic_rhs_kernel, 0, 0);
  int grid_size = (face_num + block_size - 1) / block_size;

  compute_elastic_rhs_kernel<<<grid_size, block_size>>>(
      face_num, elastic_stiffness, d_F, d_state, d_jacobian_ops, d_areas,
      d_rhs);
}

}  // namespace silk::cuda
