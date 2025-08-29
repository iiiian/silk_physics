#include "collision_narrowphase.hpp"

#include <Eigen/Geometry>
#include <cassert>
#include <tight_inclusion/ccd.hpp>
#include <tight_inclusion/interval_root_finder.hpp>

#include "logger.hpp"

namespace silk {

using Matrix34f = Eigen::Matrix<float, 3, 4>;

struct TUV {
  float t;
  float u;
  float v;
};

Collision make_pt_collision(const ObjectCollider& oa, const MeshCollider& ma,
                            const ObjectCollider& ob, const MeshCollider& mb,
                            Matrix34f p_t0, Matrix34f p_t1, Matrix34f v,
                            Eigen::Vector3f v_diff, TUV tuv, float ms,
                            float stiffness) {
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 1)) = ma.inv_mass(Eigen::seqN(0, 1));
  inv_mass(Eigen::seqN(1, 3)) = mb.inv_mass(Eigen::seqN(0, 3));

  Eigen::Array4f para = {1.0f, 1.0f - tuv.u - tuv.v, tuv.u, tuv.v};
  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f impulse_weight = para * inv_mass / denom;
  impulse_weight(0) *= -1.0f;

  Collision c;
  c.type = CollisionType::PointTriangle;
  c.toi = tuv.t;
  c.minimal_separation = ms;
  c.stiffness = stiffness;
  c.inv_mass = inv_mass;
  c.offset(0) = oa.solver_offset + 3 * ma.index(0);
  c.offset(Eigen::seqN(1, 3)) = (ob.solver_offset + 3 * mb.index.array());
  c.position_t0 = p_t0;
  c.position_t1 = p_t1;
  c.velocity_t0 = v;
  c.velocity_t1 = v + v_diff * impulse_weight.transpose();

  return c;
}

std::optional<TUV> pt_exact_uv(const Matrix34f& p, float eps) {
  auto x0 = p.col(0);  // point position
  auto x1 = p.col(1);  // triangle vertex 1 position
  auto x2 = p.col(2);  // triangle vertex 2 position
  auto x3 = p.col(3);  // triangle vertex 3 position

  Eigen::Vector3f x13 = x1 - x3;
  Eigen::Vector3f x23 = x2 - x3;
  Eigen::Vector3f x03 = x0 - x3;

  float x13dx13 = x13.squaredNorm();
  float x13dx23 = x13.dot(x23);
  float x23dx23 = x23.squaredNorm();
  float x13dx03 = x13.dot(x03);
  float x23dx03 = x23.dot(x03);
  float det = x13dx13 * x23dx23 - x13dx23 * x13dx23;

  // degenerate triangle, ignore
  float area2_eps = std::pow(eps * std::max(x13dx13, x23dx23), 2);
  if (det < area2_eps) {
    SPDLOG_DEBUG("degenerate point triangle, ignore potential collision");
    return std::nullopt;
  }

  // bary barycentric coordinate of point projection
  float b1 = (x23dx23 * x13dx03 - x13dx23 * x23dx03) / det;
  b1 = std::clamp(b1, 0.0f, 1.0f);
  float b2 = (-x13dx23 * x13dx03 + x13dx13 * x23dx03) / det;
  b2 = std::clamp(b2, 0.0f, 1.0f);

  return TUV{0.0f, b1, b2};
}

std::optional<Collision> pt_zero_toi_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, Matrix34f p_t0, Matrix34f p_t1, float dt,
    float base_stiffness, float ms, float eps) {
  auto tuv = pt_exact_uv(p_t0, eps);
  if (!tuv) {
    return std::nullopt;
  }
  float b1 = tuv->u;
  float b2 = tuv->v;

  Eigen::Vector3f proj =
      b1 * p_t0.col(1) + b2 * p_t0.col(2) + (1.0f - b1 - b2) * p_t0.col(3);
  // collision normal, point to triangle
  Eigen::Vector3f n = proj - p_t0.col(0);
  float dist2 = n.squaredNorm();

  if (dist2 < 0.25f * ms * ms) {
    SPDLOG_DEBUG("all hope is lost");
    exit(1);
  }

  float dist = std::sqrt(dist2);
  n /= dist;

  Matrix34f v = p_t1 - p_t0;
  Eigen::Vector3f v_proj =
      b1 * v.col(1) + b2 * v.col(2) + (1.0f - b1 - b2) * v.col(3);
  Eigen::Vector3f v_rel = v.col(0) - v_proj;
  float v_norm_norm = v_rel.dot(n);

  if (dist2 < ms * ms) {
    Eigen::Vector3f v_diff;
    float toi;
    if (v_norm_norm < ms) {
      v_diff = 2 * ms * n;
      toi = 0.1f;
    } else {
      Eigen::Vector3f v_norm = v_norm_norm * n;
      Eigen::Vector3f p_para = v_rel - v_norm;
      // Eigen::Vector3f v_diff =
      //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
      v_diff = 2.0f * v_norm / dt;
      toi = 0.0f;
    }

    return make_pt_collision(oa, ma, ob, mb, std::move(p_t0), std::move(p_t1),
                             std::move(v), std::move(v_diff), TUV{toi, b1, b2},
                             ms, base_stiffness);
  }

  if (v_norm_norm < 0.0f) {
    SPDLOG_DEBUG("zero toi leaving");
    return std::nullopt;
  }

  float colli_dist = std::max(dist - ms, 0.0f);

  if (v_norm_norm < colli_dist) {
    SPDLOG_DEBUG("zero toi reject");
    return std::nullopt;
  }

  float toi = colli_dist / v_norm_norm;
  SPDLOG_DEBUG("zero toi refine {}", toi);

  Eigen::Vector3f v_norm = v_norm_norm * n;
  Eigen::Vector3f p_para = v_rel - v_norm;
  // Eigen::Vector3f v_diff =
  //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  Eigen::Vector3f v_diff = 2.0f * v_norm / dt;

  return make_pt_collision(oa, ma, ob, mb, std::move(p_t0), std::move(p_t1),
                           std::move(v), std::move(v_diff), TUV{toi, b1, b2},
                           ms, base_stiffness);
}

// std::optional<Collision> edge_edge_collision(
//     float toi, const Eigen::Matrix<float, 3, 4>& position_t0,
//     const Eigen::Matrix<float, 3, 4>& position_t1,
//     const Eigen::Vector4f& weight, const CCDConfig& config) {
//   const CCDConfig& c = config;
//
//   // edge edge position at potential collision
//   Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
//   Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
//   auto x0 = p_colli.col(0);  // edge 1 vertex 1 position
//   auto x1 = p_colli.col(1);  // edge 1 vertex 2 position
//   auto x2 = p_colli.col(2);  // edge 2 vertex 1 position
//   auto x3 = p_colli.col(3);  // edge 2 vertex 2 position
//
//   Eigen::Vector3f x10 = x1 - x0;
//   Eigen::Vector3f x32 = x3 - x2;
//   Eigen::Vector3f x20 = x2 - x0;
//
//   float x10dx10 = x10.squaredNorm();
//   float x10dx32 = x10.dot(x32);
//   float x32dx32 = x32.squaredNorm();
//   float x10dx21 = x10.dot(x20);
//   float x32dx20 = x32.dot(x20);
//   float det = x10dx10 * x32dx32 - x10dx32 * x10dx32;
//
//   // zero length edge, ignore
//   if (x10dx10 < c.eps * c.eps || x32dx32 < c.eps * c.eps) {
//     return std::nullopt;
//   }
//
//   float e10_para = 0;
//   float e32_para = 0;
//
//   // find edge parameter of collision point
//
//   // parallel edge
//   float area2_eps = pow(c.eps * std::max(x10dx10, x32dx32), 2);
//   if (det < area2_eps) {
//     // test x0 against edge x2 x3
//     if (float para = x32.dot(-x20) / x32dx32; para > 0.0f && para < 1.0f) {
//       Eigen::Vector3f x0c = x2 + para * x32;
//       float dist2 = (x0 - x0c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//
//       e10_para = 0.0f;
//       e32_para = para;
//     }
//     // test x1 against edge x2 x3
//     else if (float para = x32.dot(x1 - x2) / x32dx32;
//              para > 0.0f && para < 1.0f) {
//       Eigen::Vector3f x1c = x2 + para * x32;
//       float dist2 = (x1 - x1c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//
//       e10_para = 1.0f;
//       e32_para = para;
//     }
//     // test x2 against edge x0 x1
//     else if (float para = x10.dot(x20) / x10dx10; para > 0.0f && para < 1.0f)
//     {
//       Eigen::Vector3f x2c = x0 + para * x10;
//       float dist2 = (x2 - x2c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//
//       e10_para = para;
//       e32_para = 0.0f;
//     }
//     // test x3 against edge x0 x1
//     else if (float para = x10.dot(x3 - x0) / x10dx10;
//              para > 0.0f && para < 1.0f) {
//       Eigen::Vector3f x3c = x0 + para * x10;
//       float dist2 = (x3 - x3c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//
//       e10_para = para;
//       e32_para = 1.0f;
//     }
//   }
//   // non-parallel edge, compute the closest point between two infinite line
//   // then clamp if necessary
//   else {
//     e10_para = (x32dx32 * x10dx21 - x10dx32 * x32dx20) / det;
//     bool is_e10_para_clamped = false;
//     if (e10_para < 0.0f) {
//       e10_para = 0.0f;
//       is_e10_para_clamped = true;
//     } else if (e10_para > 1.0f) {
//       e10_para = 1.0f;
//       is_e10_para_clamped = true;
//     }
//     e32_para = (x10dx32 * x10dx21 - x10dx10 * x32dx20) / det;
//     bool is_e32_para_clamped = false;
//     if (e32_para < 0.0f) {
//       e32_para = 0.0f;
//       is_e32_para_clamped = true;
//     } else if (e32_para > 1.0f) {
//       e32_para = 1.0f;
//       is_e32_para_clamped = true;
//     }
//
//     // both parameter of edge x0 x1 and edge x2 x3 are outside
//     if (is_e10_para_clamped && is_e32_para_clamped) {
//       // compute 2 possible collision point pairs then choose the closer one
//       // candidate pair a
//       Eigen::Vector3f e10c_a = x0 + e10_para * x10;
//       float e32_para_a = std::clamp(x32.dot(e10c_a - x2) / x32dx32,
//       0.0f, 1.0f); Eigen::Vector3f e32c_a = x2 + e32_para_a * x32;
//
//       // candidate pair b
//       Eigen::Vector3f e32c_b = x2 + e32_para * x32;
//       float e10_para_b = std::clamp(x10.dot(e32c_b - x0) / x10dx10,
//       0.0f, 1.0f); Eigen::Vector3f e10c_b = x0 + e10_para_b * x10;
//
//       float dist2a = (e10c_a - e32c_a).squaredNorm();
//       float dist2b = (e10c_b - e32c_b).squaredNorm();
//       if (dist2a < dist2b) {
//         if (dist2a > c.h * c.h) {
//           return std::nullopt;
//         }
//         e32_para = e32_para_a;
//       } else {
//         if (dist2b > c.h * c.h) {
//           return std::nullopt;
//         }
//         e10_para = e10_para_b;
//       }
//     }
//     // parameter of edge x0 x1 is outside
//     else if (is_e10_para_clamped) {
//       Eigen::Vector3f e10c = x0 + e10_para * x10;
//       e32_para = std::clamp(x32.dot(e10c - x2) / x32dx32, 0.0f, 1.0f);
//       Eigen::Vector3f e32c = x2 + e32_para * x32;
//       float dist2 = (e10c - e32c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//     }
//     // parameter of edge x2 x3 is outside
//     else if (is_e32_para_clamped) {
//       Eigen::Vector3f e32c = x2 + e32_para * x32;
//       e10_para = std::clamp(x10.dot(e32c - x0) / x10dx10, 0.0f, 1.0f);
//       Eigen::Vector3f e12c = x0 + e10_para * x10;
//       float dist2 = (e12c - e32c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//     }
//     // both para are inside
//     else {
//       Eigen::Vector3f e10c = x0 + e10_para * x10;
//       Eigen::Vector3f e32c = x2 + e32_para * x32;
//       float dist2 = (e10c - e32c).squaredNorm();
//       if (dist2 > c.h * c.h) {
//         return std::nullopt;
//       }
//     }
//   }
//
//   // compute relative velocity at impact
//   Eigen::Matrix<float, 3, 4> v = p_diff / c.dt;
//   Eigen::Vector3f v_e10c = e10_para * v.col(0) + (1.0f - e10_para) *
//   v.col(1); Eigen::Vector3f v_e32c = e32_para * v.col(2) + (1.0f - e32_para)
//   * v.col(3); Eigen::Vector3f v_relative = v_e10c - v_e32c;
//
//   // compute collision normal n
//   Eigen::Vector3f n;
//   // parallel edge
//   if (det < area2_eps) {
//     n = x10.cross(x10.cross(x20)).normalized();
//   }
//   // non parallel edge
//   else {
//     n = x10.cross(x32).normalized();
//   }
//
//   // compute velocity after reflection
//   Eigen::Vector3f v_normal = n.dot(v_relative) * n;
//   Eigen::Vector3f v_parallel = v_relative - v_normal;
//   float tmp0 = (weight(0) * e10_para + weight(1) * (1.0f - e10_para) +
//                 weight(2) * e32_para + weight(3) * (1.0f - e32_para));
//   if (tmp0 == 0.0f) {
//     return std::nullopt;
//   }
//
//   // Eigen::Vector4f coeff = weight.array() / tmp0;
//   Eigen::Vector4f coeff = Eigen::Vector4f::Identity();
//   coeff(0) *= -1.0f;
//   coeff(1) *= -1.0f;
//   // Eigen::Vector3f v_diff =
//   //     (2.0f - c.damping) * v_normal + (1.0f - c.friction) * v_parallel;
//   Eigen::Vector3f v_diff = (2.0f - c.damping) * v_normal;
//
//   v += v_diff * coeff.transpose();
//
//   Collision collision;
//   collision.type = CollisionType::EdgeEdge;
//   collision.toi = toi;
//   collision.position = p_colli + (1.0f - toi) * c.dt * v;
//
//   return collision;
// }

std::optional<Collision> point_triangle_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_vf_err) {
  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  // minimal separation
  float ms = std::min(oa.bbox_padding, ob.bbox_padding);

  // gather primitive position
  Matrix34f p_t0;
  Matrix34f p_t1;
  p_t0.block(0, 0, 3, 1) = ma.position_t0.block(0, 0, 3, 1);
  p_t0.block(0, 1, 3, 3) = mb.position_t0.block(0, 0, 3, 3);
  p_t1.block(0, 0, 3, 1) = ma.position_t1.block(0, 0, 3, 1);
  p_t1.block(0, 1, 3, 3) = mb.position_t1.block(0, 0, 3, 3);

  // ccd test
  std::optional<ticcd::Collision> ccd_result = ticcd::vertexFaceCCD(
      p_t0.col(0),   // point at t0
      p_t0.col(1),   // triangle vertex 0 at t0
      p_t0.col(2),   // triangle vertex 1 at t0
      p_t0.col(3),   // triangle vertex 2 at t0
      p_t1.col(0),   // point at t1
      p_t1.col(1),   // triangle vertex 0 at t1
      p_t1.col(2),   // triangle vertex 1 at t1
      p_t1.col(3),   // triangle vertex 2 at t1
      scene_vf_err,  // floating point err for the whole scene
      ms,            // minimal seperation
      tolerance,     // ticcd solving precision.
      1.0f,          // max time, we use normalized time interval [0, 1]
      max_iter,      // max ticcd iteration, set as -1 to disable
      false          // disable toi refinement if toi = 0
  );

  if (!ccd_result) {
    return std::nullopt;
  }

  float toi = ccd_result->t(0);
  if (toi == 0.0f) {
    return pt_zero_toi_collision(oa, ma, ob, mb, std::move(p_t0),
                                 std::move(p_t1), dt, base_stiffness, ms,
                                 1e-6f);
  }

  Eigen::Matrix<float, 3, 4> v = p_t1 - p_t0;
  Eigen::Matrix<float, 3, 4> p_colli = p_t0 + toi * v;

  auto tuv = pt_exact_uv(p_colli, 1e-6f);
  if (!tuv) {
    return std::nullopt;
  }
  float b1 = tuv->u;
  float b2 = tuv->v;

  p_colli = p_t0 + toi * v;

  // collision point of triangle
  Eigen::Vector3f p_tri = (1.0f - b1 - b2) * p_colli.col(1) +
                          b1 * p_colli.col(2) + b2 * p_colli.col(3);
  // velocity of collision point of traingle
  Eigen::Vector3f v_tri =
      (1.0f - b1 - b2) * v.col(1) + b1 * v.col(2) + b2 * v.col(3);

  // n is collision normal that points from point to triangle
  Eigen::Vector3f n = p_tri - p_colli.col(0);
  if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
    SPDLOG_ERROR("collision diff = 0");
    exit(1);
  }
  n.normalized();

  Eigen::Vector3f v_rel = v.col(0) - v_tri;
  float v_norm_norm = v_rel.dot(n);
  if (v_norm_norm < 0.0f) {
    SPDLOG_DEBUG("v leaving, {}", v_norm_norm);
    return std::nullopt;
  }

  Eigen::Vector3f p_diff_norm = v_norm_norm * n;
  Eigen::Vector3f p_diff_parallel = v_rel - p_diff_norm;
  // Eigen::Vector3f v_diff =
  //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  Eigen::Vector3f v_diff = 2.0f * p_diff_norm / dt;

  return make_pt_collision(oa, ma, ob, mb, std::move(p_t0), std::move(p_t1),
                           std::move(v), std::move(v_diff), TUV{toi, b1, b2},
                           ms, base_stiffness);
}

std::optional<Collision> edge_edge_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_ee_err) {
  return std::nullopt;

  // // TODO: more damping and friction avg mode
  // float damping = 0.5f * (oa.damping + ob.damping);
  // float friction = 0.5f * (oa.friction + ob.friction);
  // // minimal separation
  // float ms = std::min(oa.bbox_padding, ob.bbox_padding);
  //
  // Collision c;
  // c.position_t0.block(0, 0, 3, 2) = ma.position_t0.block(0, 0, 3, 2);
  // c.position_t0.block(0, 2, 3, 2) = mb.position_t0.block(0, 0, 3, 2);
  // c.position_t1.block(0, 0, 3, 2) = ma.position_t1.block(0, 0, 3, 2);
  // c.position_t1.block(0, 2, 3, 2) = mb.position_t1.block(0, 0, 3, 2);
  //
  // // ccd test
  // std::optional<ticcd::Collision> ccd_result = ticcd::edgeEdgeCCD(
  //     c.position_t0.col(0),  // edge a vertex 0 at t0
  //     c.position_t0.col(1),  // edge a vertex 1 at t0
  //     c.position_t0.col(2),  // edge b vertex 0 at t0
  //     c.position_t0.col(3),  // edge b vertex 1 at t0
  //     c.position_t1.col(0),  // edge a vertex 0 at t1
  //     c.position_t1.col(1),  // edge a vertex 1 at t1
  //     c.position_t1.col(2),  // edge b vertex 0 at t1
  //     c.position_t1.col(3),  // edge b vertex 1 at t1
  //     scene_ee_err,          // floating point err for the whole scene
  //     ms,                    // minimal seperation
  //     tolerance,             // ticcd solving precision
  //     1.0f,                  // max time, we use normalized time interval [0,
  //     1] max_iter,              // max ticcd iteration, set as -1 to disable
  //     true                   // enable toi refinement if toi = 0
  // );
  //
  // if (!ccd_result) {
  //   return std::nullopt;
  // }
  //
  // float toi = ccd_result->t(0);
  // float para_a = ccd_result->u(0);
  // float para_b = ccd_result->v(0);
  //
  // Eigen::Matrix<float, 3, 4> p_diff = c.position_t1 - c.position_t0;
  // Eigen::Matrix<float, 3, 4> p_colli = c.position_t0 + toi * p_diff;
  // c.velocity_t0 = p_diff / dt;
  //
  // // collision point of edge a and b
  // Eigen::Vector3f pa =
  //     (1.0f - para_a) * p_colli.col(0) + para_a * p_colli.col(1);
  // Eigen::Vector3f pb =
  //     (1.0f - para_b) * p_colli.col(2) + para_b * p_colli.col(3);
  // // collision point velocity of edge a and b
  // Eigen::Vector3f va =
  //     (1.0f - para_a) * c.velocity_t0.col(0) + para_a * c.velocity_t0.col(1);
  // Eigen::Vector3f vb =
  //     (1.0f - para_b) * c.velocity_t0.col(2) + para_b * c.velocity_t0.col(3);
  //
  // // n is collision normal that points from edge a to b
  // // Eigen::Vector3f n =
  // //     (p_colli.col(0) - p_colli.col(1)).cross(p_colli.col(2) -
  // //     p_colli.col(3));
  // // if (n.cwiseAbs().maxCoeff() < ms * 1e-6f) {
  // //   SPDLOG_WARN("parallel edge edge collision");
  // //   exit(1);
  // // }
  // // Eigen::Vector3f dir = pb - pa;
  // // if (dir(0) == 0.0f && dir(1) == 0.0f && dir(2) == 0.0f) {
  // //   SPDLOG_ERROR("zero collision direction");
  // //   exit(1);
  // // }
  // // if (n.dot(dir) < 0.0f) {
  // //   n *= -1.0f;
  // // }
  // Eigen::Vector3f n = pb - pa;
  // if (n(0) == 0.0f && n(1) == 0.0f && n(2) == 0.0f) {
  //   SPDLOG_ERROR("zero collision distance");
  //   exit(1);
  // }
  // n.normalize();
  //
  // Eigen::Vector3f v_relative = va - vb;
  // float v_normal_norm = n.dot(v_relative);
  // Eigen::Vector3f v_normal = v_normal_norm * n;
  // Eigen::Vector3f v_parallel = v_relative - v_normal;
  //
  // // total velocity chabge after collision
  // Eigen::Vector3f v_diff;
  //
  // float stationary_velocity = ms / dt;
  // // if velocity along collision normal > stationary_velocity, that means 2
  // // primitive is approaching each other.
  // if (v_normal_norm > stationary_velocity) {
  //   SPDLOG_DEBUG("approaching, n velocity length {}", v_normal_norm);
  //   // Eigen::Vector3f v_diff =
  //   //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  //   v_diff = 2.0f * v_normal;
  // }
  // // if velocity along collision < stationary_velocity but > 0, that means 2
  // // primitives is approaching each other very slowly. In this case, we gives
  // an
  // // artificial velocity to ensure separation.
  // else if (v_normal_norm >= 0.0f) {
  //   SPDLOG_DEBUG("slowly approaching, n velocity length {}", v_normal_norm);
  //   v_diff = 2.0f * stationary_velocity * n;
  // }
  // // if velocity along collision < 0. That means 2 primitives are separating.
  // else {
  //   SPDLOG_DEBUG("leaving, n velocity lenght {}", v_normal_norm);
  //   return std::nullopt;
  // }
  //
  // // compute impulse weight
  // Eigen::Array4f para = {1.0f - para_a, para_a, 1.0f - para_b, para_b};
  // Eigen::Array4f inv_mass;
  // inv_mass(Eigen::seqN(0, 2)) = ma.inv_mass(Eigen::seqN(0, 2));
  // inv_mass(Eigen::seqN(2, 2)) = mb.inv_mass(Eigen::seqN(0, 2));
  // float denom = (para.square() * inv_mass).sum();
  // Eigen::Vector4f weight = para * inv_mass / denom;
  // weight(0) *= -1.0f;
  // weight(1) *= -1.0f;
  //
  // c.velocity_t1 = c.velocity_t0 + v_diff * weight.transpose();
  // c.reflection = p_colli + (1.0f - toi) * dt * c.velocity_t1;
  //
  // c.type = CollisionType::EdgeEdge;
  // c.toi = toi;
  // c.minimal_separation = ms;
  // c.stiffness = base_stiffness;
  // c.inv_mass = inv_mass;
  // c.offset(Eigen::seqN(0, 2)) =
  //     oa.solver_offset + 3 * ma.index(Eigen::seqN(0, 2)).array();
  // c.offset(Eigen::seqN(2, 2)) =
  //     ob.solver_offset + 3 * mb.index(Eigen::seqN(0, 2)).array();
  //
  // SPDLOG_DEBUG("ee collision: {}", c.offset.transpose());
  // SPDLOG_DEBUG("edge a v0 at t0: {}", c.position_t0.col(0).transpose());
  // SPDLOG_DEBUG("edge a v1 at t0: {}", c.position_t0.col(1).transpose());
  // SPDLOG_DEBUG("edge b v0 at t0: {}", c.position_t0.col(2).transpose());
  // SPDLOG_DEBUG("edge b v1 at t0: {}", c.position_t0.col(3).transpose());
  // SPDLOG_DEBUG("edge a v0 at t1: {}", c.position_t1.col(0).transpose());
  // SPDLOG_DEBUG("edge a v1 at t1: {}", c.position_t1.col(1).transpose());
  // SPDLOG_DEBUG("edge b v0 at t1: {}", c.position_t1.col(2).transpose());
  // SPDLOG_DEBUG("edge b v1 at t1: {}", c.position_t1.col(3).transpose());
  // SPDLOG_DEBUG("edge a v0 reflected: {}", c.reflection.col(0).transpose());
  // SPDLOG_DEBUG("edge a v1 reflected: {}", c.reflection.col(1).transpose());
  // SPDLOG_DEBUG("edge b v0 reflected: {}", c.reflection.col(2).transpose());
  // SPDLOG_DEBUG("edge b v1 reflected: {}", c.reflection.col(3).transpose());
  // SPDLOG_DEBUG(
  //     "EE: toi = [{}, {}], edge a para = [{}, {}], edge b para = [{}, {}],
  //     tol "
  //     "= {}",
  //     ccd_result->t(0), ccd_result->t(1), ccd_result->u(0), ccd_result->u(1),
  //     ccd_result->v(0), ccd_result->v(1), ccd_result->tolerance);
  //
  // return c;
}

std::optional<Collision> narrow_phase(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_ee_err,
    const Eigen::Array3f& scene_vf_err) {
  // edge edge collision
  if (ma.type == MeshColliderType::Edge) {
    return edge_edge_collision(oa, ma, ob, mb, dt, base_stiffness, tolerance,
                               max_iter, scene_ee_err);
  }
  // point triangle collision: a is point and b is triangle
  else if (ma.type == MeshColliderType::Point) {
    return point_triangle_collision(oa, ma, ob, mb, dt, base_stiffness,
                                    tolerance, max_iter, scene_vf_err);
  }
  // point triangle collision: a is triangle and b is point
  else {
    return point_triangle_collision(ob, mb, oa, ma, dt, base_stiffness,
                                    tolerance, max_iter, scene_vf_err);
  }

  assert(false && "unreachable code path");
  return std::nullopt;
}

void partial_ccd_update(const Eigen::VectorXf& solver_state_t0,
                        const Eigen::VectorXf& solver_state_t1,
                        const Eigen::Array3f& scene_ee_err,
                        const Eigen::Array3f& scene_vf_err,
                        float base_stiffness, float max_stiffness,
                        float growth_factor, float tolerance, int max_iter,
                        Collision& collision) {
  auto& c = collision;

  // update primitive position
  for (int i = 0; i < 4; ++i) {
    if (c.inv_mass(i) == 0.0f) {
      continue;
    }
    c.position_t0.col(i) = solver_state_t0(Eigen::seqN(c.offset(i), 3));
  }
  for (int i = 0; i < 4; ++i) {
    if (c.inv_mass(i) == 0.0f) {
      continue;
    }
    c.position_t1.col(i) = solver_state_t1(Eigen::seqN(c.offset(i), 3));
  }

  // ccd with low max iteration
  std::optional<ticcd::Collision> ccd_result;
  if (c.type == CollisionType::PointTriangle) {
    ccd_result = ticcd::vertexFaceCCD(
        c.position_t0.col(0),  // point at t0
        c.position_t0.col(1),  // triangle vertex 0 at t0
        c.position_t0.col(2),  // triangle vertex 1 at t0
        c.position_t0.col(3),  // triangle vertex 2 at t0
        c.position_t1.col(0),  // point at t1
        c.position_t1.col(1),  // triangle vertex 0 at t1
        c.position_t1.col(2),  // triangle vertex 1 at t1
        c.position_t1.col(3),  // triangle vertex 2 at t1
        scene_vf_err,          // floating point err for the whole scene
        c.minimal_separation,  // minimal seperation
        tolerance,             // ticcd solving precision.
        1.0f,      // max time, we use normalized time interval [0, 1]
        max_iter,  // max ticcd iteration, set as -1 to disable
        false      // no toi refinement if toi = 0
    );
  } else {
    ccd_result = ticcd::edgeEdgeCCD(
        c.position_t0.col(0),  // point at t0
        c.position_t0.col(1),  // triangle vertex 0 at t0
        c.position_t0.col(2),  // triangle vertex 1 at t0
        c.position_t0.col(3),  // triangle vertex 2 at t0
        c.position_t1.col(0),  // point at t1
        c.position_t1.col(1),  // triangle vertex 0 at t1
        c.position_t1.col(2),  // triangle vertex 1 at t1
        c.position_t1.col(3),  // triangle vertex 2 at t1
        scene_ee_err,          // floating point err for the whole scene
        c.minimal_separation,  // minimal seperation
        tolerance,             // ticcd solving precision.
        1.0f,      // max time, we use normalized time interval [0, 1]
        max_iter,  // max ticcd iteration, set as -1 to disable
        false      // no toi refinement if toi = 0
    );
  }

  if (!ccd_result) {
    // SPDLOG_DEBUG("partial ccd no collision");
    c.stiffness = 0.0f;
    return;
  }

  // SPDLOG_DEBUG("partial ccd collision");

  if (c.stiffness == 0.0f) {
    c.stiffness = base_stiffness;
  } else {
    c.stiffness *= growth_factor;
    if (c.stiffness > max_stiffness) {
      c.stiffness = max_stiffness;
    }
  }
}

}  // namespace silk
