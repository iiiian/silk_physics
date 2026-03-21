#include <cassert>
#include <cuda/std/algorithm>
#include <cuda/std/numeric>

#include "backend/cuda/collision/ccd.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

class CubicPoly {
 public:
  static constexpr int BISECT_ITER = 6;

  float a;
  float b;
  float c;
  float d;

  __both__ float eval(float x) const { return ((a * x + b) * x + c) * x + d; }

  __both__ float bisect_monotone(float left, float right) const {
    assert(right > left);

    float eval_left = eval(left);
    float eval_right = eval(right);
    if (eval_left * eval_right > 0.0f) {
      return CubicPolyRoot::EMPTY;
    }

#pragma unroll
    for (int i = 0; i < BISECT_ITER; ++i) {
      float mid = 0.5f * (left + right);
      float eval_mid = eval(mid);

      if (eval_mid * eval_left <= 0.0f) {
        right = mid;
      } else {
        left = mid;
      }
    }

    return left;
  }

  __both__ CubicPolyRoot find_root(float x_min, float x_max) const {
    float t0 = b * b - 3 * a * c;
    // Extrema
    float e0 = (-b - t0) / (3 * a);
    float e1 = (-b + t0) / (3 * a);

    float t1 = ctd::min(e0, x_max);
    float t2 = ctd::max(e0, x_min);
    float t3 = ctd::min(e1, x_max);
    float t4 = ctd::max(e1, x_min);
    CubicPolyRoot root;

    // Region A.
    if (t1 > x_min) {
      root.a = bisect_monotone(x_min, t1);
    }
    // Region B.
    if (t3 > t2) {
      root.b = bisect_monotone(t2, t3);
    }
    // Region C.
    if (x_max > t4) {
      root.c = bisect_monotone(t4, x_max);
    }

    return root;
  }
};

__both__ CubicPolyRoot solve_coplaner_poly(Vec3fV x1_t0, Vec3fV x2_t0,
                                           Vec3fV x3_t0, Vec3fV x4_t0,
                                           Vec3fV x1_t1, Vec3fV x2_t1,
                                           Vec3fV x3_t1, Vec3fV x4_t1) {
  Vec3f p21 = vsub(x2_t0, x1_t0);
  Vec3f v21 = vsub(vsub(x2_t1, x1_t1), p21);
  Vec3f p31 = vsub(x3_t0, x1_t0);
  Vec3f v31 = vsub(vsub(x3_t1, x1_t1), p31);
  Vec3f p41 = vsub(x4_t0, x1_t0);
  Vec3f v41 = vsub(vsub(x4_t1, x1_t1), p41);

  Vec3f v21cv31 = cross(v21, v31);
  Vec3f p21cv31 = cross(p21, v31);
  Vec3f v21cp31 = cross(v21, p31);
  Vec3f p21cp31 = cross(p21, p31);

  float a = dot(v21cv31, v41);
  float b = dot(p21cv31, v41) + dot(v21cp31, v41) + dot(v21cv31, p41);
  float c = dot(v21cp31, p41) + dot(p21cv31, p41) + dot(p21cp31, v41);
  float d = dot(p21cp31, p41);

  CubicPoly poly{a, b, c, d};
  return poly.find_root(0.0f, 1.0f);
}

}  // namespace silk::cuda
