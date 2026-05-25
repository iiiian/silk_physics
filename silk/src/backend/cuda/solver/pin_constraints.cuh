#pragma once

#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda {

EqualityConstraints gather_pin_constraints(ObjRegistry& registry, int state_num,
                                           CudaRuntime rt);

}  // namespace silk::cuda
