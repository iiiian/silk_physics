#include "backend/cuda/solver/pin_constraints.cuh"

#include <cuda/algorithm>
#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"
#include "common/pin.hpp"

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

EqualityConstraints gather_pin_constraints(ObjRegistry& registry, int state_num,
                                           CudaRuntime rt) {
  auto global_indicator = alloc<bool>(rt, state_num, false);
  auto global_target = alloc<float>(rt, state_num, 0);

  for (uint32_t e : registry.get_entity_with_components<Pin, PhysicalState>()) {
    auto pin = registry.get<Pin>(e);
    auto state = registry.get<PhysicalState>(e);
    assert(pin && state);

    ctd::span<bool> indicator(global_indicator.data() + state->state_offset,
                              state->state_num);
    ctd::span<float> target(global_target.data() + state->state_offset,
                            state->state_num);

    // TODO: avoid host <-> device copy.
    auto indexes = host_eigen_to_device(pin->index, rt);
    auto position = host_eigen_to_device(pin->curr_position, rt);

    ctd::span<const int> perm = {};
    auto part = registry.get<MeshPartition>(e);
    if (part) {
      perm = *part->d_perm;
    }

    if (pin->is_all_pinned) {
      cu::fill_bytes(rt.stream, indicator, true);
      part->permute(position, target, rt);
    } else {
      int grid_num = div_round_up(pin->index.size(), 128);
      scatter_vertices<bool><<<grid_num, 128, 0, rt.stream.get()>>>(
          indexes, perm, true, indicator);
      scatter_vertices<float><<<grid_num, 128, 0, rt.stream.get()>>>(
          indexes, perm, position, target);
    }
  }

  return EqualityConstraints{state_num,
                             1e4,
                             1e8,
                             0.0,
                             std::move(global_indicator),
                             std::move(global_target),
                             rt};
}

}  // namespace silk::cuda::solver
