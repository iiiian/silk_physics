#pragma once

#include <Eigen/Core>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

enum class MeshColliderType { Point, Edge, Triangle };

struct MeshCollider {
  MeshColliderType type;
  // vertex index
  Vec3 index;
  // vertex inverse mass
  Vec3 inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix3f position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix3f position_t1;

  Bbox bbox;
};

}  // namespace silk::cuda
