#pragma once

#include <cuda/buffer>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda::collision {

/// @brief Detect collisions by running broad- and narrow-phase CCD.
/// @param registry ECS registry providing colliders to test and update.
/// @param dt Simulation timestep size in seconds.
/// @param collisions Collision output.
/// @param rt Cuda runtime.
/// @return Number of collisions.
ctd::span<Collision> find_collision(
    ObjRegistry& registry, float dt, int init_broadphase_cache_size,
    cu::device_buffer<Collision>& collision_storage, CudaRuntime rt);

}  // namespace silk::cuda::collision
