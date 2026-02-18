#include <cuda/buffer>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/solver/barrier_constrain.cuh"

namespace silk::cuda {

BarrierConstrain::BarrierConstrain(int state_num, CudaRuntime rt)
    : constrain_num(0),
      state_num(state_num),
      index(rt.stream, rt.mem_resource, state_num, cu::no_init),
      lhs(rt.stream, rt.mem_resource, state_num, cu::no_init),
      rhs(rt.stream, rt.mem_resource, state_num, cu::no_init) {}

}  // namespace silk::cuda
