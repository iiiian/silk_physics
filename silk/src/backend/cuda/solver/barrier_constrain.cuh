#pragma once

#include <cuda/buffer>
#include <cuda/stream>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

class BarrierConstrain {
 public:
  int constrain_num = 0;
  int state_num = 0;
  cu::device_buffer<int> index;
  cu::device_buffer<float> lhs;
  cu::device_buffer<float> rhs;

  BarrierConstrain(int state_num, CudaRuntime rt);
};

}  // namespace silk::cuda
