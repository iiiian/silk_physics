#include <cuda/std/algorithm>
#include <cuda/std/optional>
#include <cuda/std/utility>

#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

__both__ ctd::optional<ctd::pair<float, float>> exact_pt_uv(const Vec3f& x0,
                                                            const Vec3f& x1,
                                                            const Vec3f& x2,
                                                            const Vec3f& x3,
                                                            float eps) {
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

__both__ ctd::optional<ctd::pair<float, float>> exact_ee_uv(const Vec3f& x0,
                                                            const Vec3f& x1,
                                                            const Vec3f& x2,
                                                            const Vec3f& x3,
                                                            float eps) {
  auto& p1 = x0;
  auto& q1 = x1;
  auto& p2 = x2;
  auto& q2 = x3;

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
