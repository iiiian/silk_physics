#include "backend/cuda/solver/barrier_constraints.cuh"

#include <cuda/algorithm>
#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda::solver {

namespace {

template <typename T>
__global__ void scatter_vertices(ctd::span<const int> dst,
                                 ctd::span<const int> perm, T value,
                                 ctd::span<T> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= dst.size()) {
    return;
  }

  int target_vert = perm.empty() ? dst[tid] : perm[dst[tid]];
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    out[3 * target_vert + i] = value;
  }
}

template <typename T>
__global__ void scatter_vertices(ctd::span<const int> dst,
                                 ctd::span<const int> perm,
                                 ctd::span<const T> values, ctd::span<T> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= dst.size()) {
    return;
  }
  int target_vert = perm.empty() ? dst[tid] : perm[dst[tid]];
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    out[3 * target_vert + i] = values[3 * tid + i];
  }
}
}  // namespace

EqualityConstraints gather_barrier_constraints(
    int state_num,
    ctd::span<const ::silk::cuda::collision::Collision> collisions,
    CudaRuntime rt) {}

}  // namespace silk::cuda::solver
