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
  Vec4 index;

  /// Time of impact in [0,1].
  float toi;

  /// Minimum separation distance to maintain between primitives.
  float minimal_separation;

  /// Collision constraint stiffness for solver.
  float stiffness;

  /// Whether collision narrowphase use distance smaller than minimal
  /// separation to resolve time of impact.
  bool use_small_ms;

  /// Inverse mass for each vertex involved in collision. Value of 0 indicates
  /// vertex is pinned or belongs to obstacle.
  Vec4 inv_mass;

  /// Primitive vertex positions.
  /// Layout: [vertex0, vertex1, vertex2, vertex3] as row vectors.
  Mat43 position_t0;
  Mat43 position_t1;

  /// Primitive vertex velocities before/after collision.
  /// Layout: [vertex0, vertex1, vertex2, vertex3] as row vectors.
  Mat43 velocity_t0;
  Mat43 velocity_t1;
};

}  // namespace silk::cuda
