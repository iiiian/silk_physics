#include <cassert>
#include <cuda/std/algorithm>
#include <cuda/std/numeric>

#include "backend/cuda/collision/ccd.cuh"
#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

class CubicPoly {
 public:
  static constexpr int BISECT_ITER = 6;
  static constexpr float EMPTY_ROOT = ctd::numeric_limits<float>::max();

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
      return EMPTY;
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

  __both__ Vec3f find_root(float x_min, float x_max) const {
    float t0 = b * b - 3 * a * c;
    // Extrema
    float e0 = (-b - t0) / (3 * a);
    float e1 = (-b + t0) / (3 * a);

    float t1 = ctd::min(e0, x_max);
    float t2 = ctd::max(e0, x_min);
    float t3 = ctd::min(e1, x_max);
    float t4 = ctd::max(e1, x_min);
    Vec3f root;

    // Region A.
    if (t1 > x_min) {
      root(0) = bisect_monotone(x_min, t1);
    }
    // Region B.
    if (t3 > t2) {
      root(1) = bisect_monotone(t2, t3);
    }
    // Region C.
    if (x_max > t4) {
      root(2) = bisect_monotone(t4, x_max);
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

Vec3f velocity_diff(const Vec3f& v_relative,
                    const Vec3f& n, float ms,
                    float restitution, float friction) {
  float v_normal_norm = dot(v_relative, n);
  Vec3f v_normal = ax(v_normal_norm, n);
  Vec3f v_parallel = vsub(v_relative, v_normal);

  float v_diff_norm_norm;
  // Two primitives are approaching each other normally.
  if (v_normal_norm > ms) {
    v_diff_norm_norm = ax(1.0f, + restitution, v_normal_norm);
  }
  // Two primitives are approaching each other very slowly.
  // Give an artificial velocity to ensure separation.
  else {
    v_diff_norm_norm = ms;
  }

  // Static friction.
  float v_parallel_norm = norm(v_parallel);
  float static_v_parallel_norm = friction * v_diff_norm_norm;
  if (v_parallel_norm < static_v_parallel_norm) {
    return v_diff_norm_norm * n + v_parallel;
    return axpby(v_diff_norm_norm, n, 1.0f, v_parallel);
  }

  // Kinetic friction.
  return axpby(v_diff_norm_norm, n, static_v_parallel_norm / v_parallel_norm, v_parallel);
        
}

ctd::optional<Collision> pt_ccd(
 const PointCollider* point_collider,
 const TriangleCollider* triangle_collider,
 float min_sep,
 float restitution,
 float friction){

  auto& p = point_collider;
  auto& t = triangle_collider;

  Vec3f root = solve_coplaner_poly(p.v0_t0, p.v0_t1,
                                   t.v0_t0, t.v0_t1,
                                   t.v1_t0, t.v1_t1,
                                   t.v2_t0, t.v2_t1);


  auto& x0 = v0_t0;
  auto& x1 = v0_t0;
  auto& x2 = v1_t0;
  auto& x3 = v2_t0;
  Vec3f d0 = vsub(p.v0_t1, p.v0_t0);
  Vec3f d1 = vsub(t.v0_t1, t.v0_t0);
  Vec3f d2 = vsub(t.v1_t1, t.v1_t0);
  Vec3f d3 = vsub(t.v2_t1, t.v2_t0);
  for (int i=0;i<3;++i){
    if (root(i) == CubicPoly::EMPTY){ return false; }

    // discrete collision detection.
    Vec3f y0 = axpby(1.0f, x0, root(i), d0);
    Vec3f y1 = axpby(1.0f, x1, root(i), d1);
    Vec3f y2 = axpby(1.0f, x2, root(i), d2);
    Vec3f y3 = axpby(1.0f, x3, root(i), d3);
    
    auto uv = exact_pt_uv(y0, y1, y2, y3);
    if (!uv) {continue;}

    float b1 = uv->first;
    float b2 = uv->second;

    Vec3f pa = y0;
    Vec3f pb = eval_triangle_parameter(b1, b2, y1, y2, y3);
    Vec3f disp = vsub(pa, pb);
    float dist2 = dot(disp, disp);
    if (dist2 > min_sep*min_sep) {continue;}
    if (dist2 == 0.0f) {return ctd::nullopt;}

    Vec3f va = d0;
    Vec3f vb = eval_triangle_parameter(b1, b2, v1, v2, v3);
    Vec3f v_rel = vsub(va, vb);
    Vec3f n = disp / sqrt(dist2);


    // Total velocity change after collision.
    Vec3f v_diff = velocity_diff(v_rel, n, min_sep, restitution, friction);


    // Compute impulse weights.
    Vec4f para = {1.0f, 1.0f - b1 - b2, b1, b2};
    Vec4f inv_mass;
    inv_mass(0) = point_collider->inv_mass;
    inv_mass(1) = triangle_collider->inv_mass(0);
    inv_mass(2) = triangle_collider->inv_mass(1);
    inv_mass(3) = triangle_collider->inv_mass(2);

    float denom = 0.0f;
    for (int i=0;i<4;++i){
      denom += para(i) * para(i) * inv_mass(i);
    }

    Vec4f weight = ax(-1.0f / denom, vmul(para, inv_mass));
  }

}

}  // namespace silk::cuda
