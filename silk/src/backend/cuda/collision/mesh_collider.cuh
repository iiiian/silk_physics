#pragma once

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
  // position of vertex at t0, each row is a vertex
  Mat33 position_t0;
  // position of vertex at t1, each row is a vertex
  Mat33 position_t1;

  Bbox bbox;
};

}  // namespace silk::cuda
