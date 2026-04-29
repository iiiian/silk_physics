#pragma once

#include <cuda/buffer>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda::collision {

/// @brief Collision detection and resolution pipeline.
class CollisionPipeline {
 public:
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  int partial_ccd_max_iter = 32;
  int init_broadphase_ccache_size = 10000;

  // Minimum time-of-impact to prevent infinite solver loop.
  float min_toi = 0.05f;

  // TODO: use AL instead.
  // float collision_stiffness_base = 1e8f;
  // float collision_stiffness_max = 1e8f;
  // float collision_stiffness_growth = 16.0f;

  /// @brief Detect collisions by running broad- and narrow-phase CCD.
  /// @param registry ECS registry providing colliders to test and update.
  /// @param scene_bbox Axis-aligned bounds enclosing the scene for error
  /// metrics.
  /// @param dt Simulation timestep size in seconds.
  /// @return Number of collisions.
  int find_collision(Registry& registry, const Bbox& scene_bbox, float dt,
                     cu::device_buffer<Collision>& collisions, CudaRuntime rt);
};

}  // namespace silk::cuda::collision
