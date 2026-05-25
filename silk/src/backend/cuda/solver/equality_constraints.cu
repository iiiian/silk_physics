#include "backend/cuda/solver/equality_constraints.cuh"

#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

namespace {

__global__ void count_indicator(ctd::span<const bool> indicator, int* out) {
  using BlockReduce = cub::BlockReduce<int, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int active = 0;
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < indicator.size()) {
    active = static_cast<int>(indicator[tid]);
  }

  int reduced = BlockReduce(tmp).Sum(active);
  if (threadIdx.x == 0) {
    atomicAdd(out, reduced);
  }
}

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
    lagrange_mul[tid] = ctd::clamp(tmp, -max_lagrange_mul, max_lagrange_mul);
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
    rhs[tid] += penalty * target[tid] - lagrange_mul[tid];
  }
}

__global__ void accum_primal_residual_kernel(
    ctd::span<const bool> indicator, ctd::span<const float> target,
    ctd::span<const float> state, ctd::span<float> primal_norm2,
    ctd::span<float> primal_scale_x2, ctd::span<float> primal_scale_target2) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float local_norm2 = 0.0f;
  float local_scale_x2 = 0.0f;
  float local_scale_target2 = 0.0f;

  if (tid < indicator.size() && indicator[tid]) {
    float diff = state[tid] - target[tid];
    local_norm2 = diff * diff;
    local_scale_x2 = state[tid] * state[tid];
    local_scale_target2 = target[tid] * target[tid];
  }

  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;
  float reduced = BlockReduce(tmp).Sum(local_norm2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_norm2.data(), reduced);
  }
  __syncthreads();
  reduced = BlockReduce(tmp).Sum(local_scale_x2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_scale_x2.data(), reduced);
  }
  __syncthreads();
  reduced = BlockReduce(tmp).Sum(local_scale_target2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_scale_target2.data(), reduced);
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

EqualityConstraints::EqualityConstraints(int state_num, float init_penalty,
                                         float init_lagrange_mul,
                                         float max_lagrange_mul,
                                         cu::device_buffer<bool> indicator,
                                         cu::device_buffer<float> target,
                                         CudaRuntime rt)
    : state_num(std::move(state_num)),
      active_dof(0),
      indicator(std::move(indicator)),
      target(target),
      lagrange_mul(alloc<float>(rt, state_num, init_lagrange_mul)),
      init_lagrange_mul(init_lagrange_mul),
      max_lagrange_mul(max_lagrange_mul),
      init_penalty(init_penalty),
      penalty(init_penalty) {
  assert(state_num >= 0);
  assert(init_penalty > 0);
  assert(init_lagrange_mul >= 0);
  assert(max_lagrange_mul > 0 && max_lagrange_mul >= init_lagrange_mul);
  assert(this->indicator.size() == state_num);
  assert(this->target.size() == state_num);

  auto d_active_dof = alloc<int>(rt, 1, 0);
  int grid_num = div_round_up(this->indicator.size(), 128);
  count_indicator<<<grid_num, 128, 0, rt.stream.get()>>>(this->indicator,
                                                         d_active_dof.data());
  active_dof = scalar_load(d_active_dof.data(), rt);
}

void EqualityConstraints::merge(const EqualityConstraints& other,
                                CudaRuntime rt) {
  assert(state_num == other.state_num);

  int grid_num = div_round_up(state_num, 128);
  merge_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      state_num, indicator, target, lagrange_mul, other.indicator, other.target,
      other.lagrange_mul);

  auto d_active_dof = alloc<int>(rt, 1, 0);
  grid_num = div_round_up(this->indicator.size(), 128);
  count_indicator<<<grid_num, 128, 0, rt.stream.get()>>>(this->indicator,
                                                         d_active_dof.data());
  active_dof = scalar_load(d_active_dof.data(), rt);
}

void EqualityConstraints::reset_lagrange_mul(CudaRuntime rt) {
  fill_value<float>(lagrange_mul, init_lagrange_mul, rt);
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

void EqualityConstraints::accum_primal_residual(
    ctd::span<const float> state, ctd::span<float> primal_norm2,
    ctd::span<float> primal_scale_x2, ctd::span<float> primal_scale_target2,
    CudaRuntime rt) const {
  int grid_num = div_round_up(indicator.size(), 128);
  accum_primal_residual_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(
      indicator, target, state, primal_norm2, primal_scale_x2,
      primal_scale_target2);
}

void EqualityConstraints::enforce(ctd::span<float> state, CudaRuntime rt) {
  int grid_num = div_round_up(indicator.size(), 128);
  enforce_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(indicator, target,
                                                        state);
}

}  // namespace silk::cuda
