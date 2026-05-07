#pragma once

#include <cstdint>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

/// @brief Reset obstacle solver state to initial conditions.
void batch_reset_obstacle_simulation(ObjRegistry& registry);

/// @brief Prepare an obstacle entity for solver stepping.
void prepare_obstacle_simulation(ObjRegistry& registry, uint32_t& entity,
                                 CudaRuntime rt);

}  // namespace silk::cuda
