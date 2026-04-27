#pragma once

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

struct PointCollider {
  Bbox bbox;
  /// vertex index
  int index;
  /// vertex inverse mass
  float inv_mass;
  /// position of vertices at t0
  Vec3f v0_t0;
  /// position of vertices at t0
  Vec3f v0_t1;
};

struct EdgeCollider {
  Bbox bbox;
  /// vertex index
  Vec2i index;
  /// vertex inverse mass
  Vec2f inv_mass;
  /// position of vertices at t0
  Vec3f v0_t0;
  Vec3f v1_t0;
  /// position of vertices at t0
  Vec3f v0_t1;
  Vec3f v1_t1;
};

struct TriangleCollider {
  Bbox bbox;
  /// vertex index
  Vec3i index;
  /// vertex inverse mass
  Vec3f inv_mass;
  /// position of vertices at t0
  Vec3f v0_t0;
  Vec3f v1_t0;
  Vec3f v2_t0;
  /// position of vertices at t0
  Vec3f v0_t1;
  Vec3f v1_t1;
  Vec3f v2_t1;
};

}  // namespace silk::cuda
