#pragma once

#include <Eigen/Core>

namespace silk {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  float toi;
  // solver state offset for vertices,
  // if offset = -1, the vertex is pinned or from pure obstacle
  Eigen::Vector4i offset;
  Eigen::Matrix<float, 3, 4> position;
};

}  // namespace silk
