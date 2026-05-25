#pragma once

#include <cstdint>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

void assemble_obstacle(ObjRegistry& registry, uint32_t& entity, CudaRuntime rt);

}  // namespace silk::cuda
