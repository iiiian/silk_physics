#pragma once

#include <cuda/std/utility>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda::collision {

struct PointCollider {
  Bbox bbox;
  /// entity offset in global state
  int state_offset;
  /// vertex index
  int index;
  /// minimal separation distance used for CCD against this collider
  float minimal_separation;
  /// Reflection param
  float restitution;
  /// Reflection param
  float friction;
  /// vertex inverse mass
  float inv_mass;
  /// position of vertices at t0
  Vec3f v0_t0;
  /// position of vertices at t0
  Vec3f v0_t1;
};

struct EdgeCollider {
  Bbox bbox;
  /// entity offset in global state
  int state_offset;
  /// vertex index
  Vec2i index;
  /// minimal separation distance used for CCD against this collider
  float minimal_separation;
  /// Reflection param
  float restitution;
  /// Reflection param
  float friction;
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
  /// entity offset in global state
  int state_offset;
  /// vertex index
  Vec3i index;
  /// minimal separation distance used for CCD against this collider
  float minimal_separation;
  /// Reflection param
  float restitution;
  /// Reflection param
  float friction;
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

using PTCCache = ctd::pair<const TriangleCollider*, const PointCollider*>;
using EECCache = ctd::pair<const EdgeCollider*, const EdgeCollider*>;

}  // namespace silk::cuda::collision
