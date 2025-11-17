#pragma once

#include <Eigen/Core>

#include "common/handle.hpp"

namespace silk::cuda {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  Handle entity_handle_a;
  Handle entity_handle_b;

  /** Entity global state offset.
   * Value of -1 indicates vertex belongs an obstacle. */
  int state_offset_a;
  int state_offset_b;

  /**
   * Vertex index.
   */
  Eigen::Vector4i index;

  /** Time of impact in [0,1]. */
  float toi;

  /** Minimum separation distance to maintain between primitives. */
  float minimal_separation;

  /** Collision constraint stiffness for solver. */
  float stiffness;

  /** Whether collision narrowphase use distance smaller than minimal
   * separation to resolve time of impact. */
  bool use_small_ms;

  /** Inverse mass for each vertex involved in collision. Value of 0 indicates
   * vertex is pinned or belongs to obstacle */
  Eigen::Vector4f inv_mass;

  /** Primitive vertex positions.
   * Layout: [vertex0, vertex1, vertex2, vertex3] as column vectors.
   */
  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;

  /** Primitive vertex velocities before/after collision.
   * Layout: [vertex0, vertex1, vertex2, vertex3] as column vectors.
   */
  Eigen::Matrix<float, 3, 4> velocity_t0;
  Eigen::Matrix<float, 3, 4> velocity_t1;
};

}  // namespace silk::cuda
