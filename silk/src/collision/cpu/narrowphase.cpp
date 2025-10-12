#include "collision/cpu/narrowphase.hpp"

#include <Eigen/Geometry>
#include <cassert>
#include <tight_inclusion/ccd.hpp>
#include <tight_inclusion/interval_root_finder.hpp>

#include "compiler_builtin.hpp"
#include "logger.hpp"

namespace silk {

// Compute barycentric (u, v) for point–triangle projection at a candidate
// collision configuration. Return std::nullopt if invalid.
std::optional<std::pair<float, float>> exact_point_triangle_uv(
    const Eigen::Matrix<float, 3, 4>& position, float eps) {
  auto x0 = position.col(0);  // Point position.
  auto x1 = position.col(1);  // Triangle vertex 0 position.
  auto x2 = position.col(2);  // Triangle vertex 1 position.
  auto x3 = position.col(3);  // Triangle vertex 2 position.

  Eigen::Vector3f x21 = x2 - x1;
  Eigen::Vector3f x31 = x3 - x1;
  Eigen::Vector3f x01 = x0 - x1;

  float x21dx21 = x21.squaredNorm();
  float x21dx31 = x21.dot(x31);
  float x31dx31 = x31.squaredNorm();
  float x21dx01 = x21.dot(x01);
  float x31dx01 = x31.dot(x01);
  float det = x21dx21 * x31dx31 - x21dx31 * x21dx31;

  // Degenerate triangle; ignore.
  float area2_eps = std::pow(eps * std::max(x21dx21, x31dx31), 2);
  if (det < area2_eps) {
    SPDLOG_DEBUG("Ignore potential collision. Reason: degenerated triangle");
    return std::nullopt;
  }

  // Barycentric (u, v) of point projection w.r.t. (x2, x3).
  float b1 = (x31dx31 * x21dx01 - x21dx31 * x31dx01) / det;  // U.
  float b2 = (x21dx21 * x31dx01 - x21dx31 * x21dx01) / det;  // V.

  // Outside of triangle.
  if (b1 < -eps || b2 < -eps || b1 + b2 > 1.0f + eps) {
    SPDLOG_DEBUG("Ignore potential collision. Reason: outside of triangle");
    return std::nullopt;
  }

  return std::make_pair(b1, b2);
}

// Compute parameters (u, v) for the closest points on two edges at a candidate
// collision configuration. Return std::nullopt if invalid.
std::optional<std::pair<float, float>> exact_edge_edge_uv(
    const Eigen::Matrix<float, 3, 4>& position, float eps) {
  auto x0 = position.col(0);  // Edge 1 vertex 1 position.
  auto x1 = position.col(1);  // Edge 1 vertex 2 position.
  auto x2 = position.col(2);  // Edge 2 vertex 1 position.
  auto x3 = position.col(3);  // Edge 2 vertex 2 position.

  Eigen::Vector3f x10 = x1 - x0;
  Eigen::Vector3f x32 = x3 - x2;
  Eigen::Vector3f x20 = x2 - x0;

  float x10dx10 = x10.squaredNorm();
  float x10dx32 = x10.dot(x32);
  float x32dx32 = x32.squaredNorm();
  float x10dx21 = x10.dot(x20);
  float x32dx20 = x32.dot(x20);
  float det = x10dx10 * x32dx32 - x10dx32 * x10dx32;

  // Zero-length edge; ignore.
  if (x10dx10 < eps * eps * x32dx32 || x32dx32 < eps * eps * x10dx10) {
    SPDLOG_DEBUG("Ignore potential collision. Reason: zero length edge");
    return std::nullopt;
  }

  // Parallel edges. Test four vertices against the other edge.
  // Return as soon as two colliding points both lie within their edges.
  float area2_eps = pow(eps * std::max(x10dx10, x32dx32), 2);
  if (det < area2_eps) {
    // Test x0 against edge x2–x3.
    if (float para = x32.dot(-x20) / x32dx32; para > 0.0f && para < 1.0f) {
      return std::make_pair(0.0f, para);
    }
    // Test x1 against edge x2–x3.
    else if (float para = x32.dot(x1 - x2) / x32dx32;
             para > 0.0f && para < 1.0f) {
      return std::make_pair(1.0f, para);
    }
    // Test x2 against edge x0–x1.
    else if (float para = x10.dot(x20) / x10dx10; para > 0.0f && para < 1.0f) {
      return std::make_pair(para, 0.0f);
    }
    // Test x3 against edge x0–x1.
    else if (float para = x10.dot(x3 - x0) / x10dx10;
             para > 0.0f && para < 1.0f) {
      return std::make_pair(para, 1.0f);
    }

    SPDLOG_DEBUG(
        "Ignore potential collision. Reason: no colliding parallel edges");
    return std::nullopt;
  }

  // Non-parallel edges. Compute the closest points between the two infinite
  // lines. Then clamp if necessary.

  // Edge x0–x1 parameter.
  float u = (x32dx32 * x10dx21 - x10dx32 * x32dx20) / det;
  bool is_u_clamped = false;
  if (u < 0.0f) {
    u = 0.0f;
    is_u_clamped = true;
  } else if (u > 1.0f) {
    u = 1.0f;
    is_u_clamped = true;
  }

  // Edge x2–x3 parameter.
  float v = (x10dx32 * x10dx21 - x10dx10 * x32dx20) / det;
  bool is_v_clamped = false;
  if (v < 0.0f) {
    v = 0.0f;
    is_v_clamped = true;
  } else if (v > 1.0f) {
    v = 1.0f;
    is_v_clamped = true;
  }

  // Both parameters for edges x0–x1 and x2–x3 are outside.
  if (is_u_clamped && is_v_clamped) {
    // Compute two candidate collision point pairs, then choose the closer one.

    // Candidate pair A: edge x0–x1 at parameter u and its projection.
    Eigen::Vector3f e10c_a = x0 + u * x10;
    float va = std::clamp(x32.dot(e10c_a - x2) / x32dx32, 0.0f, 1.0f);
    Eigen::Vector3f e32c_a = x2 + va * x32;

    // Candidate pair B: edge x2–x3 at parameter v and its projection.
    Eigen::Vector3f e32c_b = x2 + v * x32;
    float ub = std::clamp(x10.dot(e32c_b - x0) / x10dx10, 0.0f, 1.0f);
    Eigen::Vector3f e10c_b = x0 + ub * x10;

    float dist2a = (e10c_a - e32c_a).squaredNorm();
    float dist2b = (e10c_b - e32c_b).squaredNorm();
    if (dist2a < dist2b) {
      return std::make_pair(u, va);
    } else {
      return std::make_pair(ub, v);
    }
  }
  // Parameter of edge x0–x1 is outside.
  else if (is_u_clamped) {
    Eigen::Vector3f e10c = x0 + u * x10;
    v = std::clamp(x32.dot(e10c - x2) / x32dx32, 0.0f, 1.0f);
    return std::make_pair(u, v);
  }
  // Parameter of edge x2–x3 is outside.
  else if (is_v_clamped) {
    Eigen::Vector3f e32c = x2 + v * x32;
    u = std::clamp(x10.dot(e32c - x0) / x10dx10, 0.0f, 1.0f);
    return std::make_pair(u, v);
  }
  // Both parameters are inside.
  return std::make_pair(u, v);
}

// Compute velocity difference after collision.
// If primitives are separating, return std::nullopt.
// Uses restitution and a simplified friction model where kinetic friction
// equals the maximum static friction.
std::optional<Eigen::Vector3f> velocity_diff(const Eigen::Vector3f& v_relative,
                                             const Eigen::Vector3f& n, float ms,
                                             float restitution,
                                             float friction) {
  float v_normal_norm = v_relative.dot(n);
  Eigen::Vector3f v_normal = v_normal_norm * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  if (v_normal_norm < 0.0f) {
    SPDLOG_DEBUG("leaving, n velocity norm {}", v_normal_norm);
    return std::nullopt;
  }

  float v_diff_norm_norm;
  // Two primitives are approaching each other normally.
  if (v_normal_norm > ms) {
    v_diff_norm_norm = (1.0f + restitution) * v_normal_norm;
  }
  // Two primitives are approaching each other very slowly.
  // Give an artificial velocity to ensure separation.
  else {
    v_diff_norm_norm = ms;
  }

  // Static.
  float v_parallel_norm = v_parallel.norm();
  float static_v_parallel_norm = friction * v_diff_norm_norm;
  if (v_parallel_norm < static_v_parallel_norm) {
    return v_diff_norm_norm * n + v_parallel;
  }

  // Kinetic.
  return v_diff_norm_norm * n +
         (static_v_parallel_norm / v_parallel_norm) * v_parallel;
}

std::optional<Collision> point_triangle_collision(
    const CpuObjectCollider& oa, const MeshCollider& ma,
    const CpuObjectCollider& ob, const MeshCollider& mb, float dt,
    float base_stiffness, float min_toi, float tolerance, int max_iter,
    const Eigen::Array3f& scene_vf_err) {
  // Minimal separation.
  float ms = std::min(oa.bbox_padding, ob.bbox_padding);

  Collision c;
  // Gather primitive positions.
  c.position_t0.block(0, 0, 3, 1) = ma.position_t0.block(0, 0, 3, 1);
  c.position_t0.block(0, 1, 3, 3) = mb.position_t0.block(0, 0, 3, 3);
  c.position_t1.block(0, 0, 3, 1) = ma.position_t1.block(0, 0, 3, 1);
  c.position_t1.block(0, 1, 3, 3) = mb.position_t1.block(0, 0, 3, 3);

  // CCD test.
  std::optional<ticcd::CCDResult> ccd_result = ticcd::vertexFaceCCD(
      c.position_t0.col(0),  // Point at t0.
      c.position_t0.col(1),  // Triangle vertex 0 at t0.
      c.position_t0.col(2),  // Triangle vertex 1 at t0.
      c.position_t0.col(3),  // Triangle vertex 2 at t0.
      c.position_t1.col(0),  // Point at t1.
      c.position_t1.col(1),  // Triangle vertex 0 at t1.
      c.position_t1.col(2),  // Triangle vertex 1 at t1.
      c.position_t1.col(3),  // Triangle vertex 2 at t1.
      scene_vf_err,          // Floating-point error for the whole scene.
      ms,                    // Minimal separation.
      tolerance,             // TICCD solving precision.
      1.0f,                  // Maximum time; uses normalized interval [0, 1].
      max_iter,              // Maximum TICCD iterations; set -1 to disable.
      true                   // Enable TOI refinement if TOI = 0.
  );

  if (!ccd_result) {
    return std::nullopt;
  }

  // This means two collision primitives are so close that tight inclusion CCD
  // fails to resolve TOI. It indicates the failure of other parts of the engine
  // and we have no way to recover it. So just pretend this collision doesn't
  // exist and hope that the solver can resume to a valid state magically.
  if (ccd_result->use_small_ms && ccd_result->small_ms_t(0) == 0.0f) {
    spdlog::error("Ignore potential collision. Reason: zero toi");
    return std::nullopt;
  }

  float toi = ccd_result->t(0);

  c.velocity_t0 = c.position_t1 - c.position_t0;
  Eigen::Matrix<float, 3, 4> p_colli = c.position_t0 + toi * c.velocity_t0;

  // The uv parameters from tight inclusion are imprecise and might lead to
  // invalid collision reponse. Compute exact uv at toi estimated by tight
  // inclusion CCD.
  auto uv_pair = exact_point_triangle_uv(p_colli, 1e-6f);
  if (!uv_pair) {
    return std::nullopt;
  }
  float b1 = uv_pair->first;
  float b2 = uv_pair->second;

  // Collision point on triangle.
  Eigen::Vector3f pt = (1.0f - b1 - b2) * p_colli.col(1) + b1 * p_colli.col(2) +
                       b2 * p_colli.col(3);
  // Velocity of collision point on triangle.
  Eigen::Vector3f v_pt = (1.0f - b1 - b2) * c.velocity_t0.col(1) +
                         b1 * c.velocity_t0.col(2) + b2 * c.velocity_t0.col(3);

  // Collision normal n that points from point to triangle.
  Eigen::Vector3f n = pt - p_colli.col(0);
  if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
    spdlog::error(
        "Ignore potential collision. Reason: zero collision distance");
    return std::nullopt;
  }
  n.normalize();

  Eigen::Vector3f v_relative = c.velocity_t0.col(0) - v_pt;
  // TODO: More restitution and friction averaging mode.
  float restitution = 0.5f * (oa.restitution + ob.restitution);
  float friction = 0.5f * (oa.friction + ob.friction);

  // Total velocity change after collision.
  auto v_diff = velocity_diff(v_relative, n, ms, restitution, friction);
  if (!v_diff) {
    return std::nullopt;
  }

  // Compute impulse weights.
  Eigen::Array4f para = {1.0f, 1.0f - b1 - b2, b1, b2};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 1)) = ma.inv_mass(Eigen::seqN(0, 1));
  inv_mass(Eigen::seqN(1, 3)) = mb.inv_mass(Eigen::seqN(0, 3));
  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para * inv_mass / denom;
  weight(0) *= -1.0f;

  // Compute reflected velocity.
  c.velocity_t1 = c.velocity_t0 + v_diff.value() * weight.transpose();

  // If use_small_ms is true, then at t = 0 the primitives are either very close
  // or within the minimal separation distance. To avoid the solver getting
  // stuck with zero TOI, enforce a small TOI min_toi. Likewise, even when CCD
  // does not use small_ms, ensure TOI is at least min_toi.
  if (ccd_result->use_small_ms || toi < min_toi) {
    c.toi = min_toi;
    c.use_small_ms = true;
  } else {
    c.toi = toi;
    c.use_small_ms = false;
  }

  c.type = CollisionType::PointTriangle;
  c.entity_handle_a = oa.entity_handle;
  c.entity_handle_b = ob.entity_handle;
  c.state_offset_a = oa.state_offset;
  c.state_offset_b = ob.state_offset;
  c.index(0) = ma.index(0);
  c.index(Eigen::seqN(1, 3)) = mb.index;
  c.minimal_separation = ms;
  c.stiffness = base_stiffness;
  c.inv_mass = inv_mass;

  SPDLOG_DEBUG("pt collision: {}", c.index.transpose());
  SPDLOG_DEBUG("tuv: {} {} {}", toi, b1, b2);
  SPDLOG_DEBUG("use small ms: {}", c.use_small_ms);
  SPDLOG_DEBUG("position x0 t0: {}", c.position_t0.col(0).transpose());
  SPDLOG_DEBUG("position x1 t0: {}", c.position_t0.col(1).transpose());
  SPDLOG_DEBUG("position x2 t0: {}", c.position_t0.col(2).transpose());
  SPDLOG_DEBUG("position x3 t0: {}", c.position_t0.col(3).transpose());
  SPDLOG_DEBUG("position x0 t1: {}", c.position_t1.col(0).transpose());
  SPDLOG_DEBUG("position x1 t1: {}", c.position_t1.col(1).transpose());
  SPDLOG_DEBUG("position x2 t1: {}", c.position_t1.col(2).transpose());
  SPDLOG_DEBUG("position x3 t1: {}", c.position_t1.col(3).transpose());
  SPDLOG_DEBUG("velocity x0 t0: {}", c.velocity_t0.col(0).transpose());
  SPDLOG_DEBUG("velocity x1 t0: {}", c.velocity_t0.col(1).transpose());
  SPDLOG_DEBUG("velocity x2 t0: {}", c.velocity_t0.col(2).transpose());
  SPDLOG_DEBUG("velocity x3 t0: {}", c.velocity_t0.col(3).transpose());
  SPDLOG_DEBUG("velocity x0 t1: {}", c.velocity_t1.col(0).transpose());
  SPDLOG_DEBUG("velocity x1 t1: {}", c.velocity_t1.col(1).transpose());
  SPDLOG_DEBUG("velocity x2 t1: {}", c.velocity_t1.col(2).transpose());
  SPDLOG_DEBUG("velocity x3 t1: {}", c.velocity_t1.col(3).transpose());

  return c;
}

// Perform edge–edge CCD and construct a Collision record if one occurs.
std::optional<Collision> edge_edge_collision(
    const CpuObjectCollider& oa, const MeshCollider& ma,
    const CpuObjectCollider& ob, const MeshCollider& mb, float dt,
    float base_stiffness, float min_toi, float tolerance, int max_iter,
    const Eigen::Array3f& scene_ee_err) {
  // Minimal separation.
  float ms = std::min(oa.bbox_padding, ob.bbox_padding);

  Collision c;
  c.position_t0.block(0, 0, 3, 2) = ma.position_t0.block(0, 0, 3, 2);
  c.position_t0.block(0, 2, 3, 2) = mb.position_t0.block(0, 0, 3, 2);
  c.position_t1.block(0, 0, 3, 2) = ma.position_t1.block(0, 0, 3, 2);
  c.position_t1.block(0, 2, 3, 2) = mb.position_t1.block(0, 0, 3, 2);

  // CCD test.
  std::optional<ticcd::CCDResult> ccd_result = ticcd::edgeEdgeCCD(
      c.position_t0.col(0),  // Edge A vertex 0 at t0.
      c.position_t0.col(1),  // Edge A vertex 1 at t0.
      c.position_t0.col(2),  // Edge B vertex 0 at t0.
      c.position_t0.col(3),  // Edge B vertex 1 at t0.
      c.position_t1.col(0),  // Edge A vertex 0 at t1.
      c.position_t1.col(1),  // Edge A vertex 1 at t1.
      c.position_t1.col(2),  // Edge B vertex 0 at t1.
      c.position_t1.col(3),  // Edge B vertex 1 at t1.
      scene_ee_err,          // Floating-point error for the whole scene.
      ms,                    // Minimal separation.
      tolerance,             // TICCD solving precision.
      1.0f,                  // Maximum time; uses normalized interval [0, 1].
      max_iter,              // Maximum TICCD iterations; set -1 to disable.
      true                   // Enable TOI refinement if TOI = 0.
  );

  if (!ccd_result) {
    return std::nullopt;
  }

  // This means two collision primitives are so close that tight inclusion CCD
  // fails to resolve TOI. It indicates the failure of other parts of the engine
  // and we have no way to recover it. So just pretend this collision doesn't
  // exist and hope that the solver can resume to a valid state magically.
  if (ccd_result->use_small_ms && ccd_result->small_ms_t(0) == 0.0f) {
    spdlog::error("Ignore potential collision. Reason: zero toi");
    return std::nullopt;
  }

  float toi = ccd_result->t(0);

  c.velocity_t0 = c.position_t1 - c.position_t0;
  Eigen::Matrix<float, 3, 4> p_colli = c.position_t0 + toi * c.velocity_t0;

  // The uv parameters from tight inclusion are imprecise and might lead to
  // invalid collision reponse. Compute exact uv at toi estimated by tight
  // inclusion CCD.
  auto uv_pair = exact_edge_edge_uv(p_colli, 1e-6f);
  if (!uv_pair) {
    return std::nullopt;
  }
  float para_a = uv_pair->first;
  float para_b = uv_pair->second;

  // Collision point of edges A and B.
  Eigen::Vector3f pa =
      (1.0f - para_a) * p_colli.col(0) + para_a * p_colli.col(1);
  Eigen::Vector3f pb =
      (1.0f - para_b) * p_colli.col(2) + para_b * p_colli.col(3);
  // Collision-point velocities of edges A and B.
  Eigen::Vector3f va =
      (1.0f - para_a) * c.velocity_t0.col(0) + para_a * c.velocity_t0.col(1);
  Eigen::Vector3f vb =
      (1.0f - para_b) * c.velocity_t0.col(2) + para_b * c.velocity_t0.col(3);

  // Collision normal n that points from edge A to edge B.
  Eigen::Vector3f n = pb - pa;
  if (n(0) == 0.0f && n(1) == 0.0f && n(2) == 0.0f) {
    spdlog::error(
        "Ignore potential collision. Reason: zero collision distance");
    return std::nullopt;
  }
  n.normalize();

  Eigen::Vector3f v_relative = va - vb;
  // TODO: More restitution and friction averaging mode.
  float restitution = 0.5f * (oa.restitution + ob.restitution);
  float friction = 0.5f * (oa.friction + ob.friction);

  // Total velocity change after collision.
  auto v_diff = velocity_diff(v_relative, n, ms, restitution, friction);
  if (!v_diff) {
    return std::nullopt;
  }

  // Compute impulse weights.
  Eigen::Array4f para = {1.0f - para_a, para_a, 1.0f - para_b, para_b};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 2)) = ma.inv_mass(Eigen::seqN(0, 2));
  inv_mass(Eigen::seqN(2, 2)) = mb.inv_mass(Eigen::seqN(0, 2));
  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para * inv_mass / denom;
  weight(0) *= -1.0f;
  weight(1) *= -1.0f;

  c.velocity_t1 = c.velocity_t0 + v_diff.value() * weight.transpose();

  // If use_small_ms is true, then at t = 0 the primitives are either very close
  // or within the minimal separation distance. To avoid the solver getting
  // stuck with zero TOI, enforce a small TOI min_toi. Likewise, even when CCD
  // does not use small_ms, ensure TOI is at least min_toi.
  if (ccd_result->use_small_ms || toi < min_toi) {
    c.toi = min_toi;
    c.use_small_ms = true;
  } else {
    c.toi = toi;
    c.use_small_ms = false;
  }

  c.type = CollisionType::EdgeEdge;
  c.entity_handle_a = oa.entity_handle;
  c.entity_handle_b = ob.entity_handle;
  c.state_offset_a = oa.state_offset;
  c.state_offset_b = ob.state_offset;
  c.index(Eigen::seqN(0, 2)) = ma.index(Eigen::seqN(0, 2));
  c.index(Eigen::seqN(2, 2)) = mb.index(Eigen::seqN(0, 2));
  c.minimal_separation = ms;
  c.stiffness = base_stiffness;
  c.inv_mass = inv_mass;

  SPDLOG_DEBUG("ee collision: {}", c.index.transpose());
  SPDLOG_DEBUG("tuv: {} {} {}", toi, para_a, para_b);
  SPDLOG_DEBUG("use small ms: {}", c.use_small_ms);
  SPDLOG_DEBUG("position x0 t0: {}", c.position_t0.col(0).transpose());
  SPDLOG_DEBUG("position x1 t0: {}", c.position_t0.col(1).transpose());
  SPDLOG_DEBUG("position x2 t0: {}", c.position_t0.col(2).transpose());
  SPDLOG_DEBUG("position x3 t0: {}", c.position_t0.col(3).transpose());
  SPDLOG_DEBUG("position x0 t1: {}", c.position_t1.col(0).transpose());
  SPDLOG_DEBUG("position x1 t1: {}", c.position_t1.col(1).transpose());
  SPDLOG_DEBUG("position x2 t1: {}", c.position_t1.col(2).transpose());
  SPDLOG_DEBUG("position x3 t1: {}", c.position_t1.col(3).transpose());
  SPDLOG_DEBUG("velocity x0 t0: {}", c.velocity_t0.col(0).transpose());
  SPDLOG_DEBUG("velocity x1 t0: {}", c.velocity_t0.col(1).transpose());
  SPDLOG_DEBUG("velocity x2 t0: {}", c.velocity_t0.col(2).transpose());
  SPDLOG_DEBUG("velocity x3 t0: {}", c.velocity_t0.col(3).transpose());
  SPDLOG_DEBUG("velocity x0 t1: {}", c.velocity_t1.col(0).transpose());
  SPDLOG_DEBUG("velocity x1 t1: {}", c.velocity_t1.col(1).transpose());
  SPDLOG_DEBUG("velocity x2 t1: {}", c.velocity_t1.col(2).transpose());
  SPDLOG_DEBUG("velocity x3 t1: {}", c.velocity_t1.col(3).transpose());

  return c;
}

std::optional<Collision> narrow_phase(
    const CpuObjectCollider& oa, const MeshCollider& ma,
    const CpuObjectCollider& ob, const MeshCollider& mb, float dt,
    float base_stiffness, float min_toi, float tolerance, int max_iter,
    const Eigen::Array3f& scene_ee_err, const Eigen::Array3f& scene_vf_err) {
  // Edge–edge collision.
  if (ma.type == MeshColliderType::Edge) {
    return edge_edge_collision(oa, ma, ob, mb, dt, base_stiffness, min_toi,
                               tolerance, max_iter, scene_ee_err);
  }
  // Point–triangle collision: A is point and B is triangle.
  else if (ma.type == MeshColliderType::Point) {
    return point_triangle_collision(oa, ma, ob, mb, dt, base_stiffness, min_toi,
                                    tolerance, max_iter, scene_vf_err);
  }
  // Point–triangle collision: A is triangle and B is point.
  else {
    return point_triangle_collision(ob, mb, oa, ma, dt, base_stiffness, min_toi,
                                    tolerance, max_iter, scene_vf_err);
  }

  SILK_UNREACHABLE();

  return std::nullopt;
}

void partial_ccd_update(const Eigen::VectorXf& global_state_t0,
                        const Eigen::VectorXf& global_state_t1,
                        const Eigen::Array3f& scene_ee_err,
                        const Eigen::Array3f& scene_vf_err,
                        float base_stiffness, float max_stiffness,
                        float growth_factor, float tolerance, int max_iter,
                        Collision& collision) {
  auto& c = collision;

  Eigen::Vector4i offset = 3 * c.index;
  if (c.type == CollisionType::PointTriangle) {
    offset(0) += c.state_offset_a;
    offset(1) += c.state_offset_b;
    offset(2) += c.state_offset_b;
    offset(3) += c.state_offset_b;
  } else {
    offset(0) += c.state_offset_a;
    offset(1) += c.state_offset_a;
    offset(2) += c.state_offset_b;
    offset(3) += c.state_offset_b;
  }

  // Update primitive positions.
  for (int i = 0; i < 4; ++i) {
    if (c.inv_mass(i) == 0.0f) {
      continue;
    }
    auto seq = Eigen::seqN(offset(i), 3);
    c.position_t0.col(i) = global_state_t0(seq);
    c.position_t1.col(i) = global_state_t1(seq);
  }

  // CCD with low maximum iterations.
  std::optional<ticcd::CCDResult> ccd_result;
  if (c.type == CollisionType::PointTriangle) {
    ccd_result = ticcd::vertexFaceCCD(
        c.position_t0.col(0),  // Point at t0.
        c.position_t0.col(1),  // Triangle vertex 0 at t0.
        c.position_t0.col(2),  // Triangle vertex 1 at t0.
        c.position_t0.col(3),  // Triangle vertex 2 at t0.
        c.position_t1.col(0),  // Point at t1.
        c.position_t1.col(1),  // Triangle vertex 0 at t1.
        c.position_t1.col(2),  // Triangle vertex 1 at t1.
        c.position_t1.col(3),  // Triangle vertex 2 at t1.
        scene_vf_err,          // Floating-point error for the whole scene.
        c.minimal_separation,  // Minimal separation.
        tolerance,             // TICCD solving precision.
        1.0f,                  // Maximum time; uses normalized interval [0, 1].
        max_iter,              // Maximum TICCD iterations; set -1 to disable.
        false                  // No TOI refinement if TOI = 0.
    );
  } else {
    ccd_result = ticcd::edgeEdgeCCD(
        c.position_t0.col(0),  // Edge A vertex 0 at t0.
        c.position_t0.col(1),  // Edge A vertex 1 at t0.
        c.position_t0.col(2),  // Edge B vertex 0 at t0.
        c.position_t0.col(3),  // Edge B vertex 1 at t0.
        c.position_t1.col(0),  // Edge A vertex 0 at t1.
        c.position_t1.col(1),  // Edge A vertex 1 at t1.
        c.position_t1.col(2),  // Edge B vertex 0 at t1.
        c.position_t1.col(3),  // Edge B vertex 1 at t1.
        scene_ee_err,          // Floating-point error for the whole scene.
        c.minimal_separation,  // Minimal separation.
        tolerance,             // TICCD solving precision.
        1.0f,                  // Maximum time; uses normalized interval [0, 1].
        max_iter,              // Maximum TICCD iterations; set -1 to disable.
        false                  // No TOI refinement if TOI = 0.
    );
  }

  if (!ccd_result) {
    // Partial CCD: no collision.
    c.stiffness = 0.0f;
    return;
  }

  // Partial CCD: collision detected.
  if (c.stiffness == 0.0f) {
    c.stiffness = base_stiffness;
  } else {
    c.stiffness *= growth_factor;
    if (c.stiffness > max_stiffness) {
      c.stiffness = max_stiffness;
    }
  }
}

}  // Namespace silk.
