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
    float* d_srhs)       // 32
{
  using BlockReduce = cub::BlockReduce<Vec32, 256>;
  __shared__ typename BlockReduce::TempStorage temp_storage;

  // Per-thread partial accumulator for 32 components
  Vec32 local{};

  // Compute U^T * (rhs-HX)
  // Each thread compute U(tid, 0...31) * (rhs(tid) - HX(tid)) and stored the
  // result in a thread local array of size 32.
  // Then use cub block reduction followed by device wide atomic add to
  // accumulate final result.
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < n) {
    float diff = d_rhs[tid] - d_HX[tid];
#pragma unroll
    for (int j = 0; j < 32; ++j) {
      local[j] += diff * d_U[tid + j * n];
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
}

void compute_subspace_d32_rhs(int n,
                              const float* d_U,    // n x 32, column-major
                              const float* d_HX,   // n
                              const float* d_rhs,  // n
                              float* d_srhs)       // 32
{
  constexpr int BLOCK_SIZE = 256;
  int grid_size = (n + BLOCK_SIZE + 1) / BLOCK_SIZE;
  compute_subspace_d32_rhs_kernel<<<grid_size, BLOCK_SIZE>>>(n, d_U, d_HX,
                                                             d_rhs, d_srhs);
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
