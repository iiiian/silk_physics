#pragma once

#include <cuda/buffer>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda {

/// @brief Find the minimum CCD time without constructing active contacts.
/// @param registry ECS registry providing colliders to test and update.
/// @param time_start Absolute normalized time already consumed.
/// @param max_time Maximum additional normalized time to search.
/// @param minimum_separation Scene-wide CCD separation distance.
/// @param rt Cuda runtime.
/// @return Earliest conservative TOI, or max_time if no hit exists.
float find_min_toi(ObjRegistry& registry, int init_broadphase_cache_size,
                   float time_start, float max_time, float minimum_separation,
                   float ccd_tolerance, int ccd_max_iter, CudaRuntime rt);

/// @brief Build the active contact set using DCD at one fixed time.
ctd::span<Collision> find_active_collisions(
    ObjRegistry& registry, int init_broadphase_cache_size, float time,
    float activation_distance, cu::device_buffer<Collision>& collision_storage,
    CudaRuntime rt);

}  // namespace silk::cuda
