#pragma once

#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;

  /// Entity global state offset.
  /// Value of -1 indicates vertex belongs an obstacle.
  int state_offset_a;
  int state_offset_b;

  /// Vertex index.
  Vec4i index;

  /// DCD activation distance for this contact.
  float activation_distance;

  /// Inverse mass for each vertex involved in collision. Value of 0 indicates
  /// vertex is pinned or belongs to obstacle.
  Vec4f inv_mass;

  /// Primitive vertex positions at the DCD evaluation time.
  Vec3f x0;
  Vec3f x1;
  Vec3f x2;
  Vec3f x3;

  /// Per-vertex displacements that move the contact to activation distance.
  Vec3f correction0;
  Vec3f correction1;
  Vec3f correction2;
  Vec3f correction3;
};

}  // namespace silk::cuda
