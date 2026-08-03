#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace silk::cpu {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  uint32_t entity_a;
  uint32_t entity_b;

  /// Entity global state offset.
  /// Value of -1 indicates vertex belongs an obstacle.
  int state_offset_a;
  int state_offset_b;

  /// Vertex index.
  Eigen::Vector4i index;

  /// DCD activation distance for this contact.
  float activation_distance;

  /// Collision constraint stiffness for solver.
  float stiffness;

  /// Inverse mass for each vertex involved in collision. Value of 0 indicates
  /// vertex is pinned or belongs to obstacle.
  Eigen::Vector4f inv_mass;

  /// Per-vertex displacement that moves this contact to the activation
  /// distance. Layout: [vertex0, vertex1, vertex2, vertex3].
  Eigen::Matrix<float, 3, 4> correction;
};

}  // namespace silk::cpu
