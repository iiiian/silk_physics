#pragma once

#include <Eigen/Core>

namespace silk {

enum class CollisionType { PointTriangle, EdgeEdge };

struct Collision {
  CollisionType type;
  float toi;
  // vertex inverse mass
  Eigen::Vector4f inv_mass;
  // solver state offset for vertices,
  // if offset = -1, the vertex is pinned or from pure obstacle
  Eigen::Vector4i offset;
  Eigen::Matrix<float, 3, 4> reflection;
};

}  // namespace silk
