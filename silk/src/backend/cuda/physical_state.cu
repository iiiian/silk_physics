#include <cuda_runtime_api.h>

#include <cassert>
#include <utility>
#include <vector>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/physical_state.cuh"
// #include "backend/cuda/copy_vector_like.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "common/mesh.hpp"

namespace silk::cuda {

PhysicalState::PhysicalState(int state_offset,
                             ctd::span<const float> curr_state,
                             ctd::span<const float> state_velocity,
                             CudaRuntime rt) {
  assert(curr_state.size() == state_velocity.size());
  assert(state_offset >= 0);

  this->state_offset = state_offset;
  this->state_num = curr_state.size();
  this->curr_state = vec_like_to_device(curr_state, rt);
  this->state_velocity = vec_like_to_device(state_velocity, rt);
  rt.stream.sync();
}

}  // namespace silk::cuda
