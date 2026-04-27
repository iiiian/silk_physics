#pragma once

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"
#include "common/handle.hpp"

namespace silk::cuda {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  Handle entity_handle_a;
  Handle entity_handle_b;

  /// Entity global state offset.
  /// Value of -1 indicates vertex belongs an obstacle.
  int state_offset_a;
  int state_offset_b;

  /// Vertex index.
  Vec4i index;

  /// Time of impact in [0,1].
  float toi;

  /// Minimum separation distance to maintain between primitives.
  float minimal_separation;

  /// Collision constraint stiffness for solver.
  float stiffness;

  /// Inverse mass for each vertex involved in collision. Value of 0 indicates
  /// vertex is pinned or belongs to obstacle.
  Vec4f inv_mass;

  /// Primitive vertex positions.
  Vec3f x0_t0;
  Vec3f x1_t0;
  Vec3f x2_t0;
  Vec3f x3_t0;
  Vec3f x0_t1;
  Vec3f x1_t1;
  Vec3f x2_t1;
  Vec3f x3_t1;

  /// Primitive vertex velocities.
  Vec3f v0_t0;
  Vec3f v1_t0;
  Vec3f v2_t0;
  Vec3f v3_t0;
  Vec3f v0_t1;
  Vec3f v1_t1;
  Vec3f v2_t1;
  Vec3f v3_t1;
};

}  // namespace silk::cuda
