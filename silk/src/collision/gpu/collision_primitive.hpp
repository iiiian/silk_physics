#pragma once

#include <Eigen/Core>

namespace silk::gpu {

struct EdgeCollider {
  Eigen::Vector2i index;
  Eigen::Vector2f inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix<float, 3, 2> position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix<float, 3, 2> position_t1;
};

struct TriangleCollider {
  Eigen::Vector3i index;
  Eigen::Vector3f inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix<float, 3, 3> position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix<float, 3, 3> position_t1;
};

}  // namespace silk::gpu
