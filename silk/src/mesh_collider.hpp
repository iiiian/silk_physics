#pragma once

#include <Eigen/Core>

#include "bbox.hpp"

namespace silk {

enum class MeshColliderType { Point, Edge, Triangle };

struct MeshCollider {
  MeshColliderType type;
  // vertex index
  Eigen::Vector3i index;
  // vertex inverse mass
  Eigen::Vector3f inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix3f position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix3f position_t1;
  Bbox bbox;
};

}  // namespace silk
