/** @file
 * Collision detection and response data structures for continuous collision
 * detection (CCD).
 *
 * Defines collision primitives and associated data needed for constraint-based
 * collision resolution in deformable body simulation.
 */

#pragma once

#include <Eigen/Core>

#include "handle.hpp"

namespace silk {

/** Types of collision primitives supported by the collision detection system.
 */
enum class CollisionType { PointTriangle, EdgeEdge };

/** Collision event data structure containing all information needed for
 * constraint resolution.
 */
struct Collision {
  CollisionType type;

  /** Handle to first mesh entity involved in collision. */
  Handle entity_handle_a;
  /** Handle to second mesh entity involved in collision. */
  Handle entity_handle_b;

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

  /** Object state offset for each vertex in global state vector.
   * Value of -1 indicates vertex is pinned or belongs to obstacle.
   */
  Eigen::Vector4i offset;

  /** Primitive vertex positions at start of timestep (t=0).
   * Layout: [vertex0, vertex1, vertex2, vertex3] as column vectors.
   */
  Eigen::Matrix<float, 3, 4> position_t0;
  /** Primitive vertex positions at end of timestep (t=1). */
  Eigen::Matrix<float, 3, 4> position_t1;

  /** Primitive vertex velocities before collision response. */
  Eigen::Matrix<float, 3, 4> velocity_t0;
  /** Primitive vertex velocities after collision response. */
  Eigen::Matrix<float, 3, 4> velocity_t1;
};

}  // namespace silk
