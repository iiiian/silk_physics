#include "backend/cuda/collision/ccd.cuh"

#include <cassert>
#include <cuda/std/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/numeric>

#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

class CubicPoly {
 public:
  static constexpr int BISECT_ITER = 6;
  static constexpr float EMPTY = -1.0f;

  float a;
  float b;
  float c;
  float d;

  __both__ float eval(float x) const { return ((a * x + b) * x + c) * x + d; }

  __both__ float bisect_monotone(float left, float right) const {
    if (right <= left) {
      return EMPTY;
    }

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
    float eps = 1e-8f;
    Vec3f root = {EMPTY, EMPTY, EMPTY};

    // Degenerate to quadratic or linear.
    if (ctd::abs(a) < eps) {
      if (ctd::abs(b) < eps) {
        root(0) = bisect_monotone(x_min, x_max);
        return root;
      }

      // Quadratic monotone intervals are split at the vertex.
      float e = ctd::clamp(-c / (2.0f * b), x_min, x_max);
      root(0) = bisect_monotone(x_min, e);
      root(1) = bisect_monotone(e, x_max);
      return root;
    }

    // Cubic monotone intervals are split at derivative roots.
    float disc = b * b - 3.0f * a * c;
    if (disc <= 0.0f) {
      root(0) = bisect_monotone(x_min, x_max);
      return root;
    }

    float s = ctd::sqrt(disc);
    float e0 = (-b - s) / (3.0f * a);
    float e1 = (-b + s) / (3.0f * a);
    if (e0 > e1) {
      ctd::swap(e0, e1);
    }

    e0 = ctd::clamp(e0, x_min, x_max);
    e1 = ctd::clamp(e1, x_min, x_max);
    root(0) = bisect_monotone(x_min, e0);
    root(1) = bisect_monotone(e0, e1);
    root(2) = bisect_monotone(e1, x_max);
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

__device__ bool is_initial_contact_toi(float toi) {
  constexpr float TOI_PROGRESS_EPS = 1e-6f;
  return toi <= TOI_PROGRESS_EPS;
}

__device__ ctd::optional<Collision> make_pt_collision(
    const PointCollider* p, const TriangleCollider* t, float toi, Vec3f y0,
    Vec3f y1, Vec3f y2, Vec3f y3, Vec3f d0, Vec3f d1, Vec3f d2, Vec3f d3,
    float ms, float restitution, float friction, bool is_initial_contact) {
  auto uv = exact_pt_uv(y0, y1, y2, y3, 1e-20);
  // Degenerate triangle, ignore.
  if (!uv) {
    return ctd::nullopt;
  }
  auto [u, v] = *uv;

  Vec3f pa = y0;
  Vec3f pb = eval_triangle_parameter(u, v, y1, y2, y3);
  Vec3f disp = vsub(pb, pa);
  float dist2 = dot(disp, disp);
  if (dist2 > ms * ms) {
    return ctd::nullopt;
  }
  // If for some reason penetration occurs, best we can do is ignore it.
  if (dist2 == 0.0f) {
    return ctd::nullopt;
  }

  Vec3f va = d0;
  Vec3f vb = eval_triangle_parameter(u, v, d1, d2, d3);
  Vec3f v_rel = vsub(va, vb);
  Vec3f n = ax(1.0 / sqrt(dist2), disp);

  // Compute impulse weights.
  Vec4f para = {1.0f, 1.0f - u - v, u, v};
  Vec4f inv_mass;
  inv_mass(0) = p->inv_mass;
  inv_mass(1) = t->inv_mass(0);
  inv_mass(2) = t->inv_mass(1);
  inv_mass(3) = t->inv_mass(2);

  float denom = 0.0f;
  for (int j = 0; j < 4; ++j) {
    denom += para(j) * para(j) * inv_mass(j);
  }

  Vec4f weight = ax(1.0f / denom, vmul(para, inv_mass));
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
  c.toi = toi;
  c.is_initial_contact = is_initial_contact;
  c.minimal_separation = ms;
  c.inv_mass = inv_mass;

  c.x0_t0 = y0;
  c.x1_t0 = y1;
  c.x2_t0 = y2;
  c.x3_t0 = y3;
  c.v0_t0 = d0;
  c.v1_t0 = d1;
  c.v2_t0 = d2;
  c.v3_t0 = d3;

  if (is_initial_contact) {
    float gap = ms - sqrt(dist2);
    Vec3f correction = ax(gap, n);
    c.v0_t1 = ax(weight(0), correction);
    c.v1_t1 = ax(weight(1), correction);
    c.v2_t1 = ax(weight(2), correction);
    c.v3_t1 = ax(weight(3), correction);
  } else {
    // Total velocity change after collision.
    Vec3f v_diff = velocity_diff(v_rel, n, ms, restitution, friction);
    c.v0_t1 = axpby(weight(0), v_diff, 1.0, d0);
    c.v1_t1 = axpby(weight(1), v_diff, 1.0, d1);
    c.v2_t1 = axpby(weight(2), v_diff, 1.0, d2);
    c.v3_t1 = axpby(weight(3), v_diff, 1.0, d3);
  }

  return c;
}

__device__ ctd::optional<Collision> make_ee_collision(
    const EdgeCollider* ea, const EdgeCollider* eb, float toi, Vec3f y0,
    Vec3f y1, Vec3f y2, Vec3f y3, Vec3f d0, Vec3f d1, Vec3f d2, Vec3f d3,
    float ms, float restitution, float friction, bool is_initial_contact) {
  auto uv = exact_ee_uv(y0, y1, y2, y3, 1e-20);
  // Degenerate edge, ignore.
  if (!uv) {
    return ctd::nullopt;
  }
  auto [u, v] = *uv;

  Vec3f pa = eval_edge_parameter(u, y0, y1);
  Vec3f pb = eval_edge_parameter(v, y2, y3);
  Vec3f disp = vsub(pb, pa);
  float dist2 = dot(disp, disp);
  if (dist2 > ms * ms) {
    return ctd::nullopt;
  }
  // If for some reason penetration occurs, best we can do is ignore it.
  if (dist2 == 0.0f) {
    return ctd::nullopt;
  }

  Vec3f va = eval_edge_parameter(u, d0, d1);
  Vec3f vb = eval_edge_parameter(v, d2, d3);
  Vec3f v_rel = vsub(va, vb);
  Vec3f n = ax(1.0 / sqrt(dist2), disp);

  // Compute impulse weights.
  Vec4f para = {1.0f - u, u, 1.0f - v, v};
  Vec4f inv_mass;
  inv_mass(0) = ea->inv_mass(0);
  inv_mass(1) = ea->inv_mass(1);
  inv_mass(2) = eb->inv_mass(0);
  inv_mass(3) = eb->inv_mass(1);

  float denom = 0.0f;
  for (int j = 0; j < 4; ++j) {
    denom += para(j) * para(j) * inv_mass(j);
  }

  Vec4f weight = ax(1.0f / denom, vmul(para, inv_mass));
  weight(0) *= -1.0f;
  weight(1) *= -1.0f;

  // Compute reflected velocity.
  Collision c;
  c.type = CollisionType::EdgeEdge;
  c.state_offset_a = ea->state_offset;
  c.state_offset_b = eb->state_offset;

  c.index(0) = ea->index(0);
  c.index(1) = ea->index(1);
  c.index(2) = eb->index(0);
  c.index(3) = eb->index(1);
  c.toi = toi;
  c.is_initial_contact = is_initial_contact;
  c.minimal_separation = ms;
  c.inv_mass = inv_mass;

  c.x0_t0 = y0;
  c.x1_t0 = y1;
  c.x2_t0 = y2;
  c.x3_t0 = y3;
  c.v0_t0 = d0;
  c.v1_t0 = d1;
  c.v2_t0 = d2;
  c.v3_t0 = d3;

  if (is_initial_contact) {
    float gap = ms - sqrt(dist2);
    Vec3f correction = ax(gap, n);
    c.v0_t1 = ax(weight(0), correction);
    c.v1_t1 = ax(weight(1), correction);
    c.v2_t1 = ax(weight(2), correction);
    c.v3_t1 = ax(weight(3), correction);
  } else {
    // Total velocity change after collision.
    Vec3f v_diff = velocity_diff(v_rel, n, ms, restitution, friction);
    c.v0_t1 = axpby(weight(0), v_diff, 1.0, d0);
    c.v1_t1 = axpby(weight(1), v_diff, 1.0, d1);
    c.v2_t1 = axpby(weight(2), v_diff, 1.0, d2);
    c.v3_t1 = axpby(weight(3), v_diff, 1.0, d3);
  }

  return c;
}

__device__ ctd::optional<Collision> pt_ccd(
    const PointCollider* point_collider,
    const TriangleCollider* triangle_collider) {
  auto& p = point_collider;
  auto& t = triangle_collider;
  float ms = ctd::min(p->minimal_separation, t->minimal_separation);

  float restitution = 0.5 * (p->restitution + t->restitution);
  float friction = 0.5 * (p->friction + t->friction);

  Vec3f root = solve_coplaner_poly(p->v0_t0, t->v0_t0, t->v1_t0, t->v2_t0,
                                   p->v0_t1, t->v0_t1, t->v1_t1, t->v2_t1);

  Vec3f d0 = vsub(p->v0_t1, p->v0_t0);
  Vec3f d1 = vsub(t->v0_t1, t->v0_t0);
  Vec3f d2 = vsub(t->v1_t1, t->v1_t0);
  Vec3f d3 = vsub(t->v2_t1, t->v2_t0);

  auto initial_contact =
      make_pt_collision(p, t, 0.0f, p->v0_t0, t->v0_t0, t->v1_t0, t->v2_t0,
                        d0, d1, d2, d3, ms, restitution, friction, true);
  if (initial_contact) {
    return initial_contact;
  }

  for (int i = 0; i < 3; ++i) {
    if (root(i) == CubicPoly::EMPTY) {
      continue;
    }

    // Discrete collision detection.
    Vec3f y0 = axpby(1.0f, p->v0_t0, root(i), d0);
    Vec3f y1 = axpby(1.0f, t->v0_t0, root(i), d1);
    Vec3f y2 = axpby(1.0f, t->v1_t0, root(i), d2);
    Vec3f y3 = axpby(1.0f, t->v2_t0, root(i), d3);

    auto c = make_pt_collision(p, t, root(i), y0, y1, y2, y3, d0, d1, d2,
                               d3, ms, restitution, friction,
                               is_initial_contact_toi(root(i)));
    if (c) {
      return c;
    }
  }
  return ctd::nullopt;
}

__device__ ctd::optional<Collision> ee_ccd(
    const EdgeCollider* edge_collider_a, const EdgeCollider* edge_collider_b) {
  auto& ea = edge_collider_a;
  auto& eb = edge_collider_b;
  float ms = ctd::min(ea->minimal_separation, eb->minimal_separation);

  float restitution = 0.5 * (ea->restitution + eb->restitution);
  float friction = 0.5 * (ea->friction + eb->friction);

  Vec3f root = solve_coplaner_poly(ea->v0_t0, ea->v1_t0, eb->v0_t0, eb->v1_t0,
                                   ea->v0_t1, ea->v1_t1, eb->v0_t1, eb->v1_t1);

  Vec3f d0 = vsub(ea->v0_t1, ea->v0_t0);
  Vec3f d1 = vsub(ea->v1_t1, ea->v1_t0);
  Vec3f d2 = vsub(eb->v0_t1, eb->v0_t0);
  Vec3f d3 = vsub(eb->v1_t1, eb->v1_t0);

  auto initial_contact =
      make_ee_collision(ea, eb, 0.0f, ea->v0_t0, ea->v1_t0, eb->v0_t0,
                        eb->v1_t0, d0, d1, d2, d3, ms, restitution, friction,
                        true);
  if (initial_contact) {
    return initial_contact;
  }

  for (int i = 0; i < 3; ++i) {
    if (root(i) == CubicPoly::EMPTY) {
      continue;
    }

    // Discrete collision detection.
    Vec3f y0 = axpby(1.0f, ea->v0_t0, root(i), d0);
    Vec3f y1 = axpby(1.0f, ea->v1_t0, root(i), d1);
    Vec3f y2 = axpby(1.0f, eb->v0_t0, root(i), d2);
    Vec3f y3 = axpby(1.0f, eb->v1_t0, root(i), d3);

    auto c = make_ee_collision(ea, eb, root(i), y0, y1, y2, y3, d0, d1, d2,
                               d3, ms, restitution, friction,
                               is_initial_contact_toi(root(i)));
    if (c) {
      return c;
    }
  }
  return ctd::nullopt;
}

}  // namespace silk::cuda
