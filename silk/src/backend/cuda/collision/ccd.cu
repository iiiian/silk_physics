#include <cassert>
#include <cuda/std/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/numeric>

#include "backend/cuda/collision/ccd.cuh"
#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda::collision {

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

    // Be conservative. Right might be within minimal separation distance.
    if (eval_left * eval_right > 0.0f) {
      return right;
    }

    for (int i = 0; i < BISECT_ITER; ++i) {
      float mid = 0.5f * (left + right);
      float eval_mid = eval(mid);

      if (eval_mid * eval_left <= 0.0f) {
        right = mid;
        eval_right = eval_mid;
      } else {
        left = mid;
        eval_left = eval_mid;
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

__both__ Vec3f solve_coplaner_poly(const Vec3f& x1_t0, const Vec3f& x2_t0,
                                   const Vec3f& x3_t0, const Vec3f& x4_t0,
                                   const Vec3f& x1_t1, const Vec3f& x2_t1,
                                   const Vec3f& x3_t1, const Vec3f& x4_t1) {
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

__device__ Vec3f velocity_diff(const Vec3f& v_relative, const Vec3f& n,
                               float ms, float restitution, float friction) {
  float v_normal_norm = dot(v_relative, n);
  Vec3f v_normal = ax(v_normal_norm, n);
  Vec3f v_parallel = vsub(v_relative, v_normal);

  float v_diff_norm_norm;
  // Two primitives are approaching each other normally.
  if (v_normal_norm > ms) {
    v_diff_norm_norm = (1.0 + restitution) * v_normal_norm;
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
    return axpby(v_diff_norm_norm, n, 1.0f, v_parallel);
  }

  // Kinetic friction.
  return axpby(v_diff_norm_norm, n, static_v_parallel_norm / v_parallel_norm,
               v_parallel);
}

__device__ ctd::optional<Collision> pt_ccd(
    const PointCollider* point_collider,
    const TriangleCollider* triangle_collider) {
  auto& p = point_collider;
  auto& t = triangle_collider;
  float ms = ctd::min(p->minimal_separation, t->minimal_separation);

  float restitution = 0.5 * (p->restitution + t->restitution);
  float friction = 0.5 * (p->friction + t->friction);

  Vec3f root = solve_coplaner_poly(p->v0_t0, p->v0_t1, t->v0_t0, t->v0_t1,
                                   t->v1_t0, t->v1_t1, t->v2_t0, t->v2_t1);

  Vec3f d0 = vsub(p->v0_t1, p->v0_t0);
  Vec3f d1 = vsub(t->v0_t1, t->v0_t0);
  Vec3f d2 = vsub(t->v1_t1, t->v1_t0);
  Vec3f d3 = vsub(t->v2_t1, t->v2_t0);
  for (int i = 0; i < 3; ++i) {
    // Discrete collision detection.
    Vec3f y0 = axpby(1.0f, p->v0_t0, root(i), d0);
    Vec3f y1 = axpby(1.0f, t->v0_t0, root(i), d1);
    Vec3f y2 = axpby(1.0f, t->v1_t0, root(i), d2);
    Vec3f y3 = axpby(1.0f, t->v2_t0, root(i), d3);

    auto uv = exact_pt_uv(y0, y1, y2, y3, 1e-20);
    // Degenerate triangle, ignore.
    if (!uv) {
      continue;
    }
    auto [u, v] = *uv;

    Vec3f pa = y0;
    Vec3f pb = eval_triangle_parameter(u, v, y1, y2, y3);
    Vec3f disp = vsub(pa, pb);
    float dist2 = dot(disp, disp);
    if (dist2 > ms * ms) {
      continue;
    }
    // If for some reason penetration occurs, best we can do is ignore it.
    if (dist2 == 0.0f) {
      return ctd::nullopt;
    }

    Vec3f va = d0;
    Vec3f vb = eval_triangle_parameter(u, v, d1, d2, d3);
    Vec3f v_rel = vsub(va, vb);
    Vec3f n = ax(1.0 / sqrt(dist2), disp);

    // Total velocity change after collision.
    Vec3f v_diff = velocity_diff(v_rel, n, ms, restitution, friction);

    // Compute impulse weights.
    Vec4f para = {1.0f, 1.0f - u - v, u, v};
    Vec4f inv_mass;
    inv_mass(0) = p->inv_mass;
    inv_mass(1) = t->inv_mass(0);
    inv_mass(2) = t->inv_mass(1);
    inv_mass(3) = t->inv_mass(2);

    float denom = 0.0f;
    for (int i = 0; i < 4; ++i) {
      denom += para(i) * para(i) * inv_mass(i);
    }

    Vec4f weight = ax(-1.0f / denom, vmul(para, inv_mass));
    weight(0) *= -1.0f;

    // Compute reflected velocity.
    Collision c;
    c.type = CollisionType::PointTriangle;
    c.state_offset_a = p->state_offset;
    c.state_offset_b = t->state_offset;

    c.index(0) = p->index;
    c.index(1) = t->index(0);
    c.index(2) = t->index(1);
    c.index(3) = t->index(2);
    c.toi = root(i);
    c.minimal_separation = ms;
    c.inv_mass = inv_mass;

    c.v0_t0 = d0;
    c.v1_t0 = d1;
    c.v2_t0 = d2;
    c.v3_t0 = d3;
    c.v0_t1 = axpby(weight(0), v_diff, 1.0, d0);
    c.v1_t1 = axpby(weight(1), v_diff, 1.0, d1);
    c.v2_t1 = axpby(weight(2), v_diff, 1.0, d2);
    c.v3_t1 = axpby(weight(3), v_diff, 1.0, d3);

    return c;
  }
}

__device__ ctd::optional<Collision> ee_ccd(
    const EdgeCollider* edge_collider_a, const EdgeCollider* edge_collider_b) {
  auto& ea = edge_collider_a;
  auto& eb = edge_collider_b;
  float ms = ctd::min(ea->minimal_separation, eb->minimal_separation);

  float restitution = 0.5 * (ea->restitution + eb->restitution);
  float friction = 0.5 * (ea->friction + eb->friction);

  Vec3f root = solve_coplaner_poly(ea->v0_t0, ea->v0_t1, ea->v1_t0, ea->v1_t1,
                                   eb->v0_t0, eb->v0_t1, eb->v1_t0, eb->v1_t1);

  Vec3f d0 = vsub(ea->v0_t1, ea->v0_t0);
  Vec3f d1 = vsub(ea->v1_t1, ea->v1_t0);
  Vec3f d2 = vsub(eb->v0_t1, eb->v0_t0);
  Vec3f d3 = vsub(eb->v1_t1, eb->v1_t0);
  for (int i = 0; i < 3; ++i) {
    // Discrete collision detection.
    Vec3f y0 = axpby(1.0f, ea->v0_t0, root(i), d0);
    Vec3f y1 = axpby(1.0f, ea->v1_t0, root(i), d1);
    Vec3f y2 = axpby(1.0f, eb->v0_t0, root(i), d2);
    Vec3f y3 = axpby(1.0f, eb->v1_t0, root(i), d3);

    auto uv = exact_ee_uv(y0, y1, y2, y3, 1e-20);
    // Degenerate edge, ignore.
    if (!uv) {
      continue;
    }
    auto [u, v] = *uv;

    Vec3f pa = eval_edge_parameter(u, y0, y1);
    Vec3f pb = eval_edge_parameter(v, y2, y3);
    Vec3f disp = vsub(pa, pb);
    float dist2 = dot(disp, disp);
    if (dist2 > ms * ms) {
      continue;
    }
    // If for some reason penetration occurs, best we can do is ignore it.
    if (dist2 == 0.0f) {
      return ctd::nullopt;
    }

    Vec3f va = eval_edge_parameter(u, d0, d1);
    Vec3f vb = eval_edge_parameter(v, d2, d3);
    Vec3f v_rel = vsub(va, vb);
    Vec3f n = ax(1.0 / sqrt(dist2), disp);

    // Total velocity change after collision.
    Vec3f v_diff = velocity_diff(v_rel, n, ms, restitution, friction);

    // Compute impulse weights.
    Vec4f para = {1.0f, 1.0f - u - v, u, v};
    Vec4f inv_mass;
    inv_mass(0) = ea->inv_mass(0);
    inv_mass(1) = ea->inv_mass(1);
    inv_mass(2) = eb->inv_mass(0);
    inv_mass(3) = eb->inv_mass(1);

    float denom = 0.0f;
    for (int i = 0; i < 4; ++i) {
      denom += para(i) * para(i) * inv_mass(i);
    }

    Vec4f weight = ax(-1.0f / denom, vmul(para, inv_mass));
    weight(0) *= -1.0f;
    weight(1) *= -1.0f;

    // Compute reflected velocity.
    Collision c;
    c.type = CollisionType::EdgeEdge;
    c.state_offset_a = ea->state_offset;
    c.state_offset_b = eb->state_offset;

    c.index(0) = ea->index(0);
    c.index(1) = eb->index(1);
    c.index(2) = eb->index(0);
    c.index(3) = eb->index(1);
    c.toi = root(i);
    c.minimal_separation = ms;
    c.inv_mass = inv_mass;

    c.v0_t0 = d0;
    c.v1_t0 = d1;
    c.v2_t0 = d2;
    c.v3_t0 = d3;
    c.v0_t1 = axpby(weight(0), v_diff, 1.0, d0);
    c.v1_t1 = axpby(weight(1), v_diff, 1.0, d1);
    c.v2_t1 = axpby(weight(2), v_diff, 1.0, d2);
    c.v3_t1 = axpby(weight(3), v_diff, 1.0, d3);

    return c;
  }
}

}  // namespace silk::cuda::collision
