#pragma once

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

class BarrierConstrain {
 public:
  int constrain_num = 0;
  int state_num = 0;
  Buf<int> index;
  Buf<float> lhs;
  Buf<float> rhs;
};

}  // namespace silk::cuda
