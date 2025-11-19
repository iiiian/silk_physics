#include "backend/cuda/solver/barrier_constrain.hpp"

#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

BarrierConstrain::BarrierConstrain(int state_num) {
  this->constrain_num = 0;
  this->state_num = state_num;
  CHECK_CUDA(cudaMalloc((void**)&d_index, state_num * sizeof(int)));
  CHECK_CUDA(cudaMalloc((void**)&d_lhs, state_num * sizeof(float)));
  CHECK_CUDA(cudaMalloc((void**)&d_rhs, state_num * sizeof(float)));
}

BarrierConstrain::BarrierConstrain(BarrierConstrain&& other) noexcept {
  swap(other);
}

BarrierConstrain& BarrierConstrain::operator=(
    BarrierConstrain&& other) noexcept {
  swap(other);
  return *this;
}

BarrierConstrain::~BarrierConstrain() {
  CHECK_CUDA(cudaFree(d_index));
  CHECK_CUDA(cudaFree(d_lhs));
  CHECK_CUDA(cudaFree(d_rhs));
}

void BarrierConstrain::swap(BarrierConstrain& other) noexcept {
  if (this == &other) {
    return;
  }

  std::swap(constrain_num, other.constrain_num);
  std::swap(state_num, other.state_num);
  std::swap(d_index, other.d_index);
  std::swap(d_lhs, other.d_lhs);
  std::swap(d_rhs, other.d_rhs);
}

}  // namespace silk::cuda
