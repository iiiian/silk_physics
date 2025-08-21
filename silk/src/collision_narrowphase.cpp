#include "collision_narrowphase.hpp"

#include <Eigen/Geometry>
#include <cassert>
#include <tight_inclusion/ccd.hpp>
#include <tight_inclusion/interval_root_finder.hpp>

#include "logger.hpp"

namespace silk {

std::optional<Collision> point_triangle_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_vf_err) {
  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  float h = std::min(oa.bbox_padding,
                     ob.bbox_padding);  // 5% of avg edge length

  Collision c;
  // gather primitive position
  c.position_t0.block(0, 0, 3, 1) = ma.position_t0.block(0, 0, 3, 1);
  c.position_t0.block(0, 1, 3, 3) = mb.position_t0.block(0, 0, 3, 3);
  c.position_t1.block(0, 0, 3, 1) = ma.position_t1.block(0, 0, 3, 1);
  c.position_t1.block(0, 1, 3, 3) = mb.position_t1.block(0, 0, 3, 3);

  // ccd test
  std::optional<ticcd::Collision> ccd_result = ticcd::vertexFaceCCD(
      c.position_t0.col(0),  // point at t0
      c.position_t0.col(1),  // triangle vertex 0 at t0
      c.position_t0.col(2),  // triangle vertex 1 at t0
      c.position_t0.col(3),  // triangle vertex 2 at t0
      c.position_t1.col(0),  // point at t1
      c.position_t1.col(1),  // triangle vertex 0 at t1
      c.position_t1.col(2),  // triangle vertex 1 at t1
      c.position_t1.col(3),  // triangle vertex 2 at t1
      scene_vf_err,          // floating point err for the whole scene
      h,                     // minimal seperation
      tolerance,             // ticcd solving precision.
      1.0f,                  // max time, we use normalized time interval [0, 1]
      max_iter,              // max ticcd iteration, set as -1 to disable
      false                  // no toi refinement if toi = 0
  );

  if (!ccd_result) {
    return std::nullopt;
  }

  float toi = ccd_result->t(0);
  float bary_a = ccd_result->u(0);
  float bary_b = ccd_result->v(0);

  Eigen::Matrix<float, 3, 4> p_diff = (c.position_t1 - c.position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = c.position_t0 + toi * p_diff;
  c.velocity_t0 = p_diff / dt;

  // collision point of triangle
  Eigen::Vector3f pt = (1.0f - bary_a - bary_b) * p_colli.col(1) +
                       bary_a * p_colli.col(2) + bary_b * p_colli.col(3);
  Eigen::Vector3f v_pt = (1.0f - bary_a - bary_b) * c.velocity_t0.col(1) +
                         bary_a * c.velocity_t0.col(2) +
                         bary_b * c.velocity_t0.col(3);

  Eigen::Vector3f n;
  if (toi == 0.0f) {
    n = pt - p_colli.col(0);
    if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
      SPDLOG_WARN("zero toi and zero collision distance");
      exit(1);
    }
  } else {
    n = (p_colli.col(2) - p_colli.col(1))
            .cross(p_colli.col(3) - p_colli.col(1));
    if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
      SPDLOG_WARN("degenerate triangle collision");
      return std::nullopt;
    }
  }

  n.normalize();
  Eigen::Vector3f v_relative = c.velocity_t0.col(0) - v_pt;
  float tmp = v_relative.dot(n);

  float stationary_threshold = 2.0f * h / dt;
  bool zero_toi_stationary = false;
  if (toi == 0.0f) {
    if (tmp > stationary_threshold) {
      SPDLOG_DEBUG("zero toi approaching, n velocity length {}", tmp);
    } else if (tmp < -stationary_threshold) {
      SPDLOG_DEBUG("zero toi leaving, n velocity lenght {}", tmp);
      return std::nullopt;
    } else {
      zero_toi_stationary = true;
      SPDLOG_DEBUG("zero toi stationary, n velocity length {}", tmp);
    }
  }

  // if (toi == 0.0f && tmp < 0.0f) {
  //   SPDLOG_DEBUG("false pt zero toi collision");
  //   return std::nullopt;
  // }

  Eigen::Vector3f v_normal = tmp * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute impulse weight
  Eigen::Array4f para = {1.0f, 1.0f - bary_a - bary_b, bary_a, bary_b};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 1)) = ma.inv_mass(Eigen::seqN(0, 1));
  inv_mass(Eigen::seqN(1, 3)) = mb.inv_mass(Eigen::seqN(0, 3));

  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para.array() * inv_mass.array() / denom;
  weight(0) *= -1.0f;

  // compute velocity after collision
  // Eigen::Vector3f v_diff =
  //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  Eigen::Vector3f v_diff;
  if (zero_toi_stationary) {
    v_diff = stationary_threshold * n;
  } else {
    v_diff = 2.0f * v_normal;
  }

  c.velocity_t1 = c.velocity_t0 + v_diff * weight.transpose();

  c.type = CollisionType::PointTriangle;
  c.toi = toi;
  c.minimal_separation = h;
  c.stiffness = base_stiffness;
  c.inv_mass = inv_mass;
  c.offset(0) = oa.solver_offset + 3 * ma.index(0);
  c.offset(Eigen::seqN(1, 3)) = (ob.solver_offset + 3 * mb.index.array());
  c.reflection = p_colli + (1.0f - toi) * dt * c.velocity_t1;

  SPDLOG_DEBUG("point at t0: {}", c.position_t0.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 at t0: {}", c.position_t0.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 at t0: {}", c.position_t0.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 at t0: {}", c.position_t0.col(3).transpose());
  SPDLOG_DEBUG("point at t1: {}", c.position_t1.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 at t1: {}", c.position_t1.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 at t1: {}", c.position_t1.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 at t1: {}", c.position_t1.col(3).transpose());
  SPDLOG_DEBUG("point reflected: {}", c.reflection.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 reflected: {}", c.reflection.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 reflected: {}", c.reflection.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 reflected: {}", c.reflection.col(3).transpose());
  SPDLOG_DEBUG(
      "PT: toi = [{}, {}], bary a = [{}, {}], bary b = [{}, {}], tol "
      "= {}",
      ccd_result->t(0), ccd_result->t(1), ccd_result->u(0), ccd_result->u(1),
      ccd_result->v(0), ccd_result->v(1), ccd_result->tolerance);

  return c;
}

std::optional<Collision> edge_edge_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_ee_err) {
  return std::nullopt;

  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  float h = 0.2f * std::min(oa.bbox_padding,
                            ob.bbox_padding);  // 5% of avg edge length

  Collision c;
  c.position_t0.block(0, 0, 3, 2) = ma.position_t0.block(0, 0, 3, 2);
  c.position_t0.block(0, 2, 3, 2) = mb.position_t0.block(0, 0, 3, 2);
  c.position_t1.block(0, 0, 3, 2) = ma.position_t1.block(0, 0, 3, 2);
  c.position_t1.block(0, 2, 3, 2) = mb.position_t1.block(0, 0, 3, 2);

  // ccd test
  std::optional<ticcd::Collision> ccd_result = ticcd::edgeEdgeCCD(
      c.position_t0.col(0),  // edge a vertex 0 at t0
      c.position_t0.col(1),  // edge a vertex 1 at t0
      c.position_t0.col(2),  // edge b vertex 0 at t0
      c.position_t0.col(3),  // edge b vertex 1 at t0
      c.position_t1.col(0),  // edge a vertex 0 at t1
      c.position_t1.col(1),  // edge a vertex 1 at t1
      c.position_t1.col(2),  // edge b vertex 0 at t1
      c.position_t1.col(3),  // edge b vertex 1 at t1
      scene_ee_err,          // floating point err for the whole scene
      h,                     // minimal seperation
      tolerance,             // ticcd solving precision
      1.0f,                  // max time, we use normalized time interval [0, 1]
      max_iter,              // max ticcd iteration, set as -1 to disable
      false                  // no toi refinement if toi = 0
  );

  if (!ccd_result) {
    return std::nullopt;
  }

  float toi = ccd_result->t(0);
  float para_a = ccd_result->u(0);
  float para_b = ccd_result->v(0);

  Eigen::Matrix<float, 3, 4> p_diff = (c.position_t1 - c.position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = c.position_t0 + toi * p_diff;
  c.velocity_t0 = p_diff / dt;

  // collision point of edge a and b
  Eigen::Vector3f pa =
      (1.0f - para_a) * p_colli.col(0) + para_a * p_colli.col(1);
  Eigen::Vector3f pb =
      (1.0f - para_b) * p_colli.col(2) + para_b * p_colli.col(3);
  Eigen::Vector3f va =
      (1.0f - para_a) * c.velocity_t0.col(0) + para_a * c.velocity_t0.col(1);
  Eigen::Vector3f vb =
      (1.0f - para_b) * c.velocity_t0.col(2) + para_b * c.velocity_t0.col(3);

  Eigen::Vector3f n = (pb - pa).normalized();
  assert((n(0) != 0.0f || n(1) != 0.0f || n(2) != 0.0f) &&
         "collision normal is zero");

  // TODO: handle parallel edges at toi

  Eigen::Vector3f v_relative = va - vb;
  float tmp = n.dot(v_relative);

  if (toi == 0.0f && tmp < 0.0f) {
    SPDLOG_DEBUG("false ee zero toi collision");
    return std::nullopt;
  }

  Eigen::Vector3f v_normal = tmp * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute impulse weight
  Eigen::Array4f para = {1.0f - para_a, para_a, 1.0f - para_b, para_b};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 2)) = ma.inv_mass(Eigen::seqN(0, 2));
  inv_mass(Eigen::seqN(2, 2)) = mb.inv_mass(Eigen::seqN(0, 2));

  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para * inv_mass / denom;
  weight(0) *= -1.0f;
  weight(1) *= -1.0f;

  // compute velocity after collision
  // Eigen::Vector3f v_diff =
  //     (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  Eigen::Vector3f v_diff = 2.0f * v_normal;
  c.velocity_t1 = c.velocity_t0 + v_diff * weight.transpose();

  c.type = CollisionType::EdgeEdge;
  c.toi = toi;
  c.minimal_separation = h;
  c.stiffness = base_stiffness;
  c.inv_mass = inv_mass;
  c.offset(Eigen::seqN(0, 2)) =
      oa.solver_offset + 3 * ma.index(Eigen::seqN(0, 2)).array();
  c.offset(Eigen::seqN(2, 2)) =
      ob.solver_offset + 3 * mb.index(Eigen::seqN(0, 2)).array();
  c.reflection = p_colli + (1.0f - toi) * dt * c.velocity_t0;

  SPDLOG_DEBUG("edge a v0 at t0: {}", c.position_t0.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 at t0: {}", c.position_t0.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 at t0: {}", c.position_t0.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 at t0: {}", c.position_t0.col(3).transpose());
  SPDLOG_DEBUG("edge a v0 at t1: {}", c.position_t1.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 at t1: {}", c.position_t1.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 at t1: {}", c.position_t1.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 at t1: {}", c.position_t1.col(3).transpose());
  SPDLOG_DEBUG("edge a v0 reflected: {}", c.reflection.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 reflected: {}", c.reflection.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 reflected: {}", c.reflection.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 reflected: {}", c.reflection.col(3).transpose());
  SPDLOG_DEBUG(
      "EE: toi = [{}, {}], edge a para = [{}, {}], edge b para = [{}, {}], tol "
      "= {}",
      ccd_result->t(0), ccd_result->t(1), ccd_result->u(0), ccd_result->u(1),
      ccd_result->v(0), ccd_result->v(1), ccd_result->tolerance);

  return c;
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

  auto& scene_err = (collision.type == CollisionType::PointTriangle)
                        ? scene_vf_err
                        : scene_ee_err;

  // ccd with low max iteration
  std::optional<ticcd::Collision> ccd_result = ticcd::vertexFaceCCD(
      c.position_t0.col(0),  // point at t0
      c.position_t0.col(1),  // triangle vertex 0 at t0
      c.position_t0.col(2),  // triangle vertex 1 at t0
      c.position_t0.col(3),  // triangle vertex 2 at t0
      c.position_t1.col(0),  // point at t1
      c.position_t1.col(1),  // triangle vertex 0 at t1
      c.position_t1.col(2),  // triangle vertex 1 at t1
      c.position_t1.col(3),  // triangle vertex 2 at t1
      scene_err,             // floating point err for the whole scene
      c.minimal_separation,  // minimal seperation
      tolerance,             // ticcd solving precision.
      1.0f,                  // max time, we use normalized time interval [0, 1]
      max_iter,              // max ticcd iteration, set as -1 to disable
      false                  // no toi refinement if toi = 0
  );

  if (!ccd_result) {
    c.stiffness = 0.0f;
    return;
  }

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
