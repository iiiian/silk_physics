#include <cuda_runtime.h>

#include <cub/block/block_reduce.cuh>
#include <cuda/std/array>

#include "backend/cuda/solver/inexact_solver_kernel.hpp"

namespace silk::cuda {

using Vec32 = ::cuda::std::array<float, 32>;

struct Vec32Plus {
  __device__ Vec32 operator()(const Vec32& a, const Vec32& b) const {
    Vec32 out;
#pragma unroll
    for (int i = 0; i < 32; ++i) {
      out[i] = a[i] + b[i];
    }
    return out;
  }
};

__global__ void compute_subspace_d32_rhs_kernel(
    int n,
    const float* d_U,    // n x 32, column-major
    const float* d_HX,   // n
    const float* d_rhs,  // n
    const float* d_dH,   // n
    const float* d_X,    // n
    float* d_srhs)       // 32
{
  using BlockReduce = cub::BlockReduce<Vec32, 256>;
  __shared__ typename BlockReduce::TempStorage temp_storage;

  // Per-thread partial accumulator for 32 components
  Vec32 local{};

  // Compute U^T * (rhs - H X - ΔH X). All threads participate in BlockReduce.
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < n) {
    float dh = d_dH ? d_dH[tid] : 0.0f;
    float diff = d_rhs[tid] - d_HX[tid] - dh * d_X[tid];
#pragma unroll
    for (int j = 0; j < 32; ++j) {
      local[j] += diff * d_U[tid + j * n];
    }
  }

  // Block-wide reduction of Vec
  Vec32 block_sum = BlockReduce(temp_storage).Reduce(local, Vec32Plus{});

  // Accumulate block contributions into srhs
  if (threadIdx.x == 0) {
#pragma unroll
    for (int j = 0; j < 32; ++j) {
      atomicAdd(&d_srhs[j], block_sum[j]);
    }
  }
}

void compute_subspace_d32_rhs(int n,
                              const float* d_U,        // n x 32, column-major
                              const float* d_HX,       // n
                              const float* d_rhs,      // n
                              const float* d_delta_H,  // n
                              const float* d_X,        // n
                              float* d_srhs)           // 32
{
  constexpr int BLOCK_SIZE = 256;
  int grid_size = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
  compute_subspace_d32_rhs_kernel<<<grid_size, BLOCK_SIZE>>>(
      n, d_U, d_HX, d_rhs, d_delta_H, d_X, d_srhs);
}

__global__ void compute_subspace_d32_UdHU_kernel(
    int constrain_num, const int* d_index, const float* d_dH_global,
    const float* d_U_row_major, int state_offset, int state_size,
    float* d_UDHU) {
  using BlockReduce = cub::BlockReduce<Vec32, 256>;
  __shared__ typename BlockReduce::TempStorage temp_storage;

  int p = blockIdx.y;  // row in subspace (0..31)
  if (p >= 32) {
    return;
  }

  Vec32 local{};
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = gridDim.x * blockDim.x;

  // Each thread processes up to two active DOFs (two rows of U) per loop
  // iteration for better memory throughput.
  for (int k = tid * 2; k < constrain_num; k += stride * 2) {
#pragma unroll
    for (int kk = 0; kk < 2; ++kk) {
      int idx_k = k + kk;
      if (idx_k >= constrain_num) {
        break;
      }

      int dof_idx = d_index[idx_k];
      if (dof_idx < state_offset || dof_idx >= state_offset + state_size) {
        continue;
      }

      int local_idx = dof_idx - state_offset;
      float dh = d_dH_global[dof_idx];
      if (dh == 0.0f) {
        continue;
      }

      const float* u_row = d_U_row_major + static_cast<size_t>(local_idx) * 32;
      float scale = dh * u_row[p];

#pragma unroll
      for (int q = 0; q < 32; ++q) {
        local[q] += scale * u_row[q];
      }
    }
  }

  Vec32 row_sum = BlockReduce(temp_storage).Reduce(local, Vec32Plus{});
  if (threadIdx.x == 0) {
#pragma unroll
    for (int q = 0; q < 32; ++q) {
      atomicAdd(&d_UDHU[p + q * 32], row_sum[q]);
    }
  }
}

void compute_subspace_d32_UdHU(int constrain_num, const int* d_index,
                               const float* d_dH_global,
                               const float* d_U_row_major, int state_offset,
                               int state_size, float* d_UDHU) {
  if (constrain_num == 0) {
    return;
  }
  constexpr int BLOCK_SIZE = 256;
  int grid_x = (constrain_num + 2 * BLOCK_SIZE - 1) / (2 * BLOCK_SIZE);
  if (grid_x <= 0) {
    grid_x = 1;
  }
  dim3 grid(grid_x, 32, 1);
  compute_subspace_d32_UdHU_kernel<<<grid, BLOCK_SIZE>>>(
      constrain_num, d_index, d_dH_global, d_U_row_major, state_offset,
      state_size, d_UDHU);
}

__global__ void project_subspace_d32_sol_kernel(int n, const float* d_U,
                                                const float* d_X,
                                                const float* ssol, float* out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float accu = 0.0f;
  if (tid < n) {
    for (int i = 0; i < 32; ++i) {
      accu += d_U[tid + i * n] * ssol[i];
    }
    out[tid] = d_X[tid] + accu;
  }
}

void project_subspace_d32_sol(int n, const float* d_U, const float* d_X,
                              const float* ssol, float* out) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     project_subspace_d32_sol_kernel, 0, 0);
  int grid_size = (n + block_size - 1) / block_size;

  project_subspace_d32_sol_kernel<<<grid_size, block_size>>>(n, d_U, d_X, ssol,
                                                             out);
}

}  // namespace silk::cuda
