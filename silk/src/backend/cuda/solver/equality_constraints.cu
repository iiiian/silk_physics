#include "backend/cuda/solver/equality_constraints.cuh"

#include <cuda/algorithm>
#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda::solver {

namespace {

__global__ void merge_kernel(int state_num, ctd::span<bool> indicator,
                             ctd::span<float> target,
                             ctd::span<float> lagrange_mul,
                             ctd::span<const bool> indicator_other,
                             ctd::span<const float> target_other,
                             ctd::span<const float> lagrange_mul_other) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= state_num) {
    return;
  }

  if (indicator_other[tid]) {
    indicator[tid] = true;
    target[tid] = target_other[tid];
    lagrange_mul[tid] = lagrange_mul_other[tid];
  }
}

__global__ void update_lagrange_mul_kernel(ctd::span<const bool> indicator,
                                           ctd::span<const float> target,
                                           ctd::span<const float> state,
                                           float penalty,
                                           float max_lagrange_mul,
                                           ctd::span<float> lagrange_mul) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= state.size()) {
    return;
  }

  if (indicator[tid]) {
    float tmp = lagrange_mul[tid] + penalty * (state[tid] - target[tid]);
    lagrange_mul[tid] = min(max_lagrange_mul, tmp);
  }
}

__global__ void eval_kernel(ctd::span<const bool> indicator,
                            ctd::span<const float> target, float penalty,
                            ctd::span<const float> lagrange_mul,
                            ctd::span<float> lhs_diag, ctd::span<float> rhs) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= indicator.size()) {
    return;
  }

  if (indicator[tid]) {
    lhs_diag[tid] += penalty;
    rhs[tid] += penalty * target[tid] + lagrange_mul[tid];
  }
}

__global__ void enforce_kernel(ctd::span<const bool> indicator,
                               ctd::span<const float> target,
                               ctd::span<float> state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= indicator.size()) {
    return;
  }

  if (indicator[tid]) {
    state[tid] = target[tid];
  }
}

}  // namespace

void EqualityConstraints::merge(const EqualityConstraints& other,
                                CudaRuntime rt) {
  assert(state_num == other.state_num);

  int grid_num = div_round_up(state_num, 128);
  merge_kernel<<<grid_num, 120, 0, rt.stream.get()>>>(
      state_num, indicator, target, lagrange_mul, other.indicator, other.target,
      other.lagrange_mul);
}

void EqualityConstraints::reset_lagrange_mul(CudaRuntime rt) {
  cu::fill_bytes(rt.stream, lagrange_mul, init_lagrange_mul);
}

void EqualityConstraints::update_lagrange_mul(ctd::span<const float> state,
                                              CudaRuntime rt) {
  int grid_num = div_round_up(state.size(), 128);
  update_lagrange_mul_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      indicator, target, state, penalty, max_lagrange_mul, lagrange_mul);
}

void EqualityConstraints::eval(ctd::span<float> lhs_diag, ctd::span<float> rhs,
                               CudaRuntime rt) const {
  int grid_num = div_round_up(indicator.size(), 128);
  eval_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      indicator, target, penalty, lagrange_mul, lhs_diag, rhs);
}

void EqualityConstraints::enforce(ctd::span<float> state, CudaRuntime rt) {
  int grid_num = div_round_up(indicator.size(), 128);
  enforce_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(indicator, target,
                                                        state);
}

}  // namespace silk::cuda::solver
