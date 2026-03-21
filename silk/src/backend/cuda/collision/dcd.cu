#include <cuda/std/algorithm>
#include <cuda/std/optional>
#include <cuda/std/utility>

#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Compute barycentric (u, v) of the point projected on the triangle.
__both__ ctd::optional<ctd::pair<float, float>> exact_point_triangle_uv(
    Mat34fV position, float eps) {
  auto x0 = position.col(0);  // Point position.
  auto x1 = position.col(1);  // Triangle vertex 0 position.
  auto x2 = position.col(2);  // Triangle vertex 1 position.
  auto x3 = position.col(3);  // Triangle vertex 2 position.

  Vec3f x21 = vsub(x2, x1);
  Vec3f x31 = vsub(x3, x1);
  Vec3f x01 = vsub(x0, x1);

  float x21dx21 = dot(x21, x21);
  float x21dx31 = dot(x21, x31);
  float x31dx31 = dot(x31, x31);
  float x21dx01 = dot(x21, x01);
  float x31dx01 = dot(x31, x01);
  float det = x21dx21 * x31dx31 - x21dx31 * x21dx31;

  // Degenerate triangle; ignore.
  float area_eps = eps * ctd::max(x21dx21, x31dx31);
  if (det < area_eps * area_eps) {
    return ctd::nullopt;
  }

  // Barycentric (u, v) of point projection w.r.t. (x2, x3).
  float b1 = (x31dx31 * x21dx01 - x21dx31 * x31dx01) / det;  // U.
  float b2 = (x21dx21 * x31dx01 - x21dx31 * x21dx01) / det;  // V.

  return ctd::make_pair(b1, b2);
}

/// @brief Compute parameters (u, v) for the closest points on two edges.
///
/// See Real-Time Collision Detection ch. 5.1.9
__both__ ctd::optional<ctd::pair<float, float>> exact_edge_edge_uv(
    Mat34fV position, float eps) {
  auto p1 = position.col(0);
  auto q1 = position.col(1);
  auto p2 = position.col(2);
  auto q2 = position.col(3);
  Vec3f d1 = vsub(q1, p1);
  Vec3f d2 = vsub(q2, p2);
  Vec3f r = vsub(p1, p2);
  float a = dot(d1, d1);
  float b = dot(d1, d2);
  float c = dot(d1, r);
  float e = dot(d2, d2);
  float f = dot(d2, r);

  // If either edge is degenerated, skip.
  if (a < eps || e < eps) {
    return ctd::nullopt;
  }

  float denom = a * e - b * b;

  float u, v;

  // Non parallel.
  if (denom > eps) {
    u = ctd::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
  }
  // Parallel.
  else {
    u = 0.0f;
  }

  v = (b * u + f) / e;
  if (v < 0.0f) {
    v = 0.0f;
    u = ctd::clamp(-c / 1, 0.0f, 1.0f);
  } else if (v > 1.0f) {
    v = 1.0f;
    u = ctd::clamp((b - c) / a, 0.0f, 1.0f);
  }
}

}  // namespace silk::cuda
