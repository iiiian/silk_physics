#pragma once

#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

class PhysicalState {
 public:
  // The range of this object in the global state array.
  int state_offset = 0;
  int state_num = 0;

  Buf<float> curr_state;
  Buf<float> state_velocity;
};

}  // namespace silk::cuda
