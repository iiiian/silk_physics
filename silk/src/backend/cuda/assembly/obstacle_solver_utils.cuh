#pragma once

#include <cstdint>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

/// @brief Prepare an obstacle entity for solver stepping.
void prepare_obstacle_simulation(ObjRegistry& registry, uint32_t& entity,
                                 CudaRuntime rt);

}  // namespace silk::cuda
