#pragma once

#include <cstdint>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

void assemble_cloth(ObjRegistry& registry, uint32_t entity, float dt,
                    int state_offset, CudaRuntime rt);

}  // namespace silk::cuda
