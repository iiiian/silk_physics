#include "backend/cuda/solver/barrier_constraints.cuh"

#include <cuda/std/span>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda {

namespace {

__device__ void compute_barrier(int state_offset, int index, float inv_mass,
                                Vec3f position, Vec3f correction,
                                ctd::span<bool> indicator,
                                ctd::span<float> target_sum,
                                ctd::span<int> target_count) {
  if (inv_mass == 0) {
    return;
  }

  Vec3f target = vadd(position, correction);

  // TODO: small ms specialization?
  int offset = state_offset + 3 * index;
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    indicator[offset + i] = true;
  }
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    atomicAdd(target_sum.data() + offset + i, target(i));
    atomicAdd(target_count.data() + offset + i, 1);
  }
}

__global__ void compute_all_barriers(ctd::span<const Collision> collisions,
                                     ctd::span<bool> indicator,
                                     ctd::span<float> target_sum,
                                     ctd::span<int> target_count) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= collisions.size()) {
    return;
  }

  auto& c = collisions[tid];

  if (c.type == CollisionType::PointTriangle) {
    compute_barrier(c.state_offset_a, c.index(0), c.inv_mass(0), c.x0,
                    c.correction0, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_b, c.index(1), c.inv_mass(1), c.x1,
                    c.correction1, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_b, c.index(2), c.inv_mass(2), c.x2,
                    c.correction2, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_b, c.index(3), c.inv_mass(3), c.x3,
                    c.correction3, indicator, target_sum, target_count);
  } else {
    compute_barrier(c.state_offset_a, c.index(0), c.inv_mass(0), c.x0,
                    c.correction0, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_a, c.index(1), c.inv_mass(1), c.x1,
                    c.correction1, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_b, c.index(2), c.inv_mass(2), c.x2,
                    c.correction2, indicator, target_sum, target_count);
    compute_barrier(c.state_offset_b, c.index(3), c.inv_mass(3), c.x3,
                    c.correction3, indicator, target_sum, target_count);
  }
}

__global__ void normalize_barrier_targets(ctd::span<const int> target_count,
                                          ctd::span<float> target) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= target.size()) {
    return;
  }

  int count = target_count[tid];
  if (count > 0) {
    target[tid] /= count;
  }
}
}  // namespace

EqualityConstraints gather_barrier_constraints(
    int state_num, ctd::span<const Collision> collisions, CudaRuntime rt) {
  constexpr float INIT_LAGRANGE_MUL = 0.0;
  constexpr float MAX_LAGRANGE_MUL = 1e10;
  constexpr float INIT_PENALTY = 1e4;

  auto indicator = alloc<bool>(rt, state_num, false);
  auto target = alloc<float>(rt, state_num, 0);
  auto target_count = alloc<int>(rt, state_num, 0);
  if (collisions.empty()) {
    return EqualityConstraints{state_num,
                               INIT_PENALTY,
                               INIT_LAGRANGE_MUL,
                               MAX_LAGRANGE_MUL,
                               std::move(indicator),
                               std::move(target),
                               rt};
  }

  int grid_num = div_round_up(collisions.size(), 128);
  compute_all_barriers<<<grid_num, 128, 0, rt.stream.get()>>>(
      collisions, indicator, target, target_count);
  grid_num = div_round_up(state_num, 128);
  normalize_barrier_targets<<<grid_num, 128, 0, rt.stream.get()>>>(target_count,
                                                                   target);
  return EqualityConstraints{state_num,
                             INIT_PENALTY,
                             INIT_LAGRANGE_MUL,
                             MAX_LAGRANGE_MUL,
                             std::move(indicator),
                             std::move(target),
                             rt};
}

}  // namespace silk::cuda
