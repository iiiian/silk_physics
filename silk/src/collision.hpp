#pragma once

#include <Eigen/Core>

#include "handle.hpp"

namespace silk {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  Handle entity_handle_a;
  Handle entity_handle_b;
  float toi;
  float minimal_separation;
  // collision constrain stiffness
  float stiffness;
  bool use_small_ms;
  // vertex inverse mass
  Eigen::Vector4f inv_mass;
  // solver state offset for vertices,
  // if offset = -1, the vertex is pinned or from pure obstacle
  Eigen::Vector4i offset;
  // primitive position for ccd query
  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  // primitive velocity before/after collision
  Eigen::Matrix<float, 3, 4> velocity_t0;
  Eigen::Matrix<float, 3, 4> velocity_t1;
};

}  // namespace silk
