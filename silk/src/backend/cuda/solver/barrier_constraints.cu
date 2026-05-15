#include "backend/cuda/solver/barrier_constraints.cuh"

#include <cuda/std/span>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda::solver {

using Collision = ::silk::cuda::collision::Collision;

namespace {

__device__ void compute_barrier(int state_offset, int index, float toi,
                                float inv_mass, Vec3f pos, Vec3f vel_before,
                                Vec3f vel_after, ctd::span<bool> indicator,
                                ctd::span<float> target) {
  if (inv_mass == 0) {
    return;
  }

  Vec3f reflection = axpby(1.0, pos, toi, vel_before);
  reflection = axpby(1.0, reflection, 1.0 - toi, vel_after);

  // TODO: small ms specialization?
  int offset = state_offset + 3 * index;
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    indicator[offset + i] = true;
  }
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    target[offset + i] = reflection(i);
  }
}

__global__ void compute_all_barriers(ctd::span<const Collision> collisions,
                                     ctd::span<bool> indicator,
                                     ctd::span<float> target) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= collisions.size()) {
    return;
  }

  auto& c = collisions[tid];

  // Vertex 0.
  compute_barrier(c.state_offset_a, c.index(0), c.toi, c.inv_mass(0), c.x0_t0,
                  c.v0_t0, c.v0_t1, indicator, target);
  // Vertex 1.
  compute_barrier(c.state_offset_a, c.index(1), c.toi, c.inv_mass(1), c.x1_t0,
                  c.v1_t0, c.v1_t1, indicator, target);
  // Vertex 2.
  compute_barrier(c.state_offset_b, c.index(2), c.toi, c.inv_mass(2), c.x2_t0,
                  c.v2_t0, c.v2_t1, indicator, target);
  // Vertex 3.
  compute_barrier(c.state_offset_b, c.index(3), c.toi, c.inv_mass(3), c.x3_t0,
                  c.v3_t0, c.v3_t1, indicator, target);
}
}  // namespace

EqualityConstraints gather_barrier_constraints(
    int state_num, ctd::span<const Collision> collisions, CudaRuntime rt) {
  constexpr float INIT_LAGRANGE_MUL = 0.0;
  constexpr float MAX_LAGRANGE_MUL = 1e10;
  constexpr float INIT_PENALTY = 1e4;

  auto indicator = alloc<bool>(rt, state_num, false);
  auto target = alloc<float>(rt, state_num, 0);
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
      collisions, indicator, target);
  return EqualityConstraints{state_num,
                             INIT_PENALTY,
                             INIT_LAGRANGE_MUL,
                             MAX_LAGRANGE_MUL,
                             std::move(indicator),
                             std::move(target),
                             rt};
}

}  // namespace silk::cuda::solver
