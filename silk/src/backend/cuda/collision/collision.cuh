#pragma once

#include <cstdint>

#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;

  /// ECS entity handle. For point-triangle, A is the point and B is the
  /// triangle.
  uint32_t object_id_a;
  uint32_t object_id_b;

  /// Entity global state offset.
  /// Value of -1 indicates vertex belongs an obstacle.
  int state_offset_a;
  int state_offset_b;

  /// Vertex index.
  Vec4i index;

  /// DCD activation distance for this contact.
  float activation_distance;

  /// Coulomb friction coefficient combined from the two primitives.
  float friction;

  /// Frozen contact frame and scalar contact Jacobian coefficients. The
  /// affine gap is sum_i coefficient(i) * x_i - activation_distance * normal.
  Vec3f normal;
  Vec4f coefficient;

  /// Inverse mass for each vertex involved in collision. Value of 0 indicates
  /// vertex is pinned or belongs to obstacle.
  Vec4f inv_mass;

  /// Primitive vertex positions at the DCD evaluation time.
  Vec3f x0;
  Vec3f x1;
  Vec3f x2;
  Vec3f x3;

  /// Prescribed obstacle displacement in the contact variable.
  Vec3f prescribed_displacement;

  /// Per-vertex displacements that move the contact to activation distance.
  Vec3f correction0;
  Vec3f correction1;
  Vec3f correction2;
  Vec3f correction3;
};

}  // namespace silk::cuda
