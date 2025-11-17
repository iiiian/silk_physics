#include "backend/cuda/object_state.hpp"

#include <cuda_runtime_api.h>

#include <utility>

#include "backend/cuda/copy_vector_like.hpp"
#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

ObjectState::ObjectState(int state_offset, const Eigen::VectorXf& curr_state,
                         const Eigen::VectorXf& state_velocity) {
  assert(curr_state.size() == state_velocity.size());
  assert(state_offset >= 0);

  this->state_num = curr_state.size();
  this->state_offset = state_offset;
  d_curr_state = host_eigen_to_device(curr_state);
  d_state_velocity = host_eigen_to_device(state_velocity);
}

ObjectState::ObjectState(ObjectState&& other) noexcept { swap(other); }

ObjectState& ObjectState::operator=(ObjectState&& other) noexcept {
  swap(other);
  return *this;
}

ObjectState::~ObjectState() {
  CHECK_CUDA(cudaFree(d_curr_state));
  CHECK_CUDA(cudaFree(d_state_velocity));
}

void ObjectState::swap(ObjectState& other) noexcept {
  if (this == &other) {
    return;
  }
  std::swap(d_curr_state, other.d_curr_state);
  std::swap(d_state_velocity, other.d_state_velocity);
}

}  // namespace silk::cuda
