#pragma once

#include <cuda/buffer>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda::collision {

/// @brief Collision detection and resolution pipeline.
class CollisionPipeline {
 public:
  int init_ccache_size = 10000;

  // TODO: use AL instead.
  // float collision_stiffness_base = 1e8f;
  // float collision_stiffness_max = 1e8f;
  // float collision_stiffness_growth = 16.0f;

  /// @brief Detect collisions by running broad- and narrow-phase CCD.
  /// @param registry ECS registry providing colliders to test and update.
  /// @param dt Simulation timestep size in seconds.
  /// @param collisions Collision output.
  /// @param rt Cuda runtime.
  /// @return Number of collisions.
  int find_collision(Registry& registry, float dt,
                     cu::device_buffer<Collision>& collisions, CudaRuntime rt);
};

}  // namespace silk::cuda::collision
