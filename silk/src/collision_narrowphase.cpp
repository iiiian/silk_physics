#include "collision_narrowphase.hpp"

#include <Eigen/Geometry>
#include <cassert>
#include <tight_inclusion/ccd.hpp>
#include <tight_inclusion/interval_root_finder.hpp>

#include "logger.hpp"

namespace silk {

std::optional<Collision> point_triangle_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, const Eigen::Array3f& scene_vf_err) {
  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  float h =
      std::min(oa.bbox_padding, ob.bbox_padding);  // 5% of avg edge length

  // gather primitive position
  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  position_t0.block(0, 0, 3, 1) = ma.position_t0.block(0, 0, 3, 1);
  position_t0.block(0, 1, 3, 3) = mb.position_t0.block(0, 0, 3, 3);
  position_t1.block(0, 0, 3, 1) = ma.position_t1.block(0, 0, 3, 1);
  position_t1.block(0, 1, 3, 3) = mb.position_t1.block(0, 0, 3, 3);

  // ccd test
  std::optional<ticcd::Collision> c = ticcd::vertexFaceCCD(
      position_t0.col(0),  // point at t0
      position_t0.col(1),  // triangle vertex 0 at t0
      position_t0.col(2),  // triangle vertex 1 at t0
      position_t0.col(3),  // triangle vertex 2 at t0
      position_t1.col(0),  // point at t1
      position_t1.col(1),  // triangle vertex 0 at t1
      position_t1.col(2),  // triangle vertex 1 at t1
      position_t1.col(3),  // triangle vertex 2 at t1
      scene_vf_err,        // floating point err for the whole scene
      h,                   // minimal seperation
      1e-6f,  // ticcd solving precision. set to 1e-6f as recommended in README
      1.0f,   // max time, we use normalized time interval [0, 1]
      -1,     // max ticcd iteration, set as -1 to disable
      false   // no toi refinement if toi = 0
  );

  if (!c) {
    return std::nullopt;
  }

  float toi = c->t(0);
  float bary_a = c->u(0);
  float bary_b = c->v(0);

  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  Eigen::Matrix<float, 3, 4> v = p_diff / dt;

  // collision point of triangle
  Eigen::Vector3f pt = (1.0f - bary_a - bary_b) * p_colli.col(1) +
                       bary_a * p_colli.col(2) + bary_b * p_colli.col(3);
  Eigen::Vector3f v_pt = (1.0f - bary_a - bary_b) * v.col(1) +
                         bary_a * v.col(2) + bary_b * v.col(3);

  Eigen::Vector3f n =
      (p_colli.col(2) - p_colli.col(1)).cross(p_colli.col(3) - p_colli.col(1));
  if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
    SPDLOG_WARN("degenerate triangle collision");
    return std::nullopt;
  }
  n.normalize();

  Eigen::Vector3f v_relative = v.col(0) - v_pt;
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute impulse weight
  // fill your code here
  Eigen::Array4f para = {1.0f, 1.0f - bary_a - bary_b, bary_a, bary_b};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 1)) = ma.inv_mass(Eigen::seqN(0, 1));
  inv_mass(Eigen::seqN(1, 3)) = mb.inv_mass(Eigen::seqN(0, 3));

  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para.array() * inv_mass.array() / denom;
  weight(0) *= -1.0f;

  // compute velocity after collision
  Eigen::Vector3f v_diff =
      (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  v += v_diff * weight.transpose();

  Collision collision;
  collision.type = CollisionType::PointTriangle;
  collision.toi = toi;
  collision.offset(0) = oa.solver_offset + 3 * ma.index(0);
  collision.offset(Eigen::seq(1, 3)) = ob.solver_offset + 3 * mb.index.array();
  collision.reflection = p_colli + (1.0f - toi) * dt * v;

  SPDLOG_DEBUG("point at t0: {}", position_t0.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 at t0: {}", position_t0.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 at t0: {}", position_t0.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 at t0: {}", position_t0.col(3).transpose());
  SPDLOG_DEBUG("point at t1: {}", position_t1.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 at t1: {}", position_t1.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 at t1: {}", position_t1.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 at t1: {}", position_t1.col(3).transpose());
  SPDLOG_DEBUG("point reflected: {}", collision.reflection.col(0).transpose());
  SPDLOG_DEBUG("triangle v1 reflected: {}",
               collision.reflection.col(1).transpose());
  SPDLOG_DEBUG("triangle v2 reflected: {}",
               collision.reflection.col(2).transpose());
  SPDLOG_DEBUG("triangle v3 reflected: {}",
               collision.reflection.col(3).transpose());
  SPDLOG_DEBUG(
      "PT: toi = [{}, {}], bary a = [{}, {}], bary b = [{}, {}], tol "
      "= {}",
      c->t(0), c->t(1), c->u(0), c->u(1), c->v(0), c->v(1), c->tolerance);

  return collision;
}

std::optional<Collision> edge_edge_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, const Eigen::Array3f& scene_ee_err) {
  // TODO: more damping and friction avg mode
  float damping = 0.5f * (oa.damping + ob.damping);
  float friction = 0.5f * (oa.friction + ob.friction);
  float h =
      std::min(oa.bbox_padding, ob.bbox_padding);  // 5% of avg edge length

  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  position_t0.block(0, 0, 3, 2) = ma.position_t0.block(0, 0, 3, 2);
  position_t0.block(0, 2, 3, 2) = mb.position_t0.block(0, 0, 3, 2);
  position_t1.block(0, 0, 3, 2) = ma.position_t1.block(0, 0, 3, 2);
  position_t1.block(0, 2, 3, 2) = mb.position_t1.block(0, 0, 3, 2);

  // ccd test
  std::optional<ticcd::Collision> c = ticcd::edgeEdgeCCD(
      position_t0.col(0),  // edge a vertex 0 at t0
      position_t0.col(1),  // edge a vertex 1 at t0
      position_t0.col(2),  // edge b vertex 0 at t0
      position_t0.col(3),  // edge b vertex 1 at t0
      position_t1.col(0),  // edge a vertex 0 at t1
      position_t1.col(1),  // edge a vertex 1 at t1
      position_t1.col(2),  // edge b vertex 0 at t1
      position_t1.col(3),  // edge b vertex 1 at t1
      scene_ee_err,        // floating point err for the whole scene
      h,                   // minimal seperation
      1e-6f,  // ticcd solving precision. set to 1e-6f as recommended in README
      1.0f,   // max time, we use normalized time interval [0, 1]
      -1,     // max ticcd iteration, set as -1 to disable
      false   // no toi refinement if toi = 0
  );

  if (!c) {
    return std::nullopt;
  }

  float toi = c->t(0);
  float para_a = c->u(0);
  float para_b = c->v(0);

  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  Eigen::Matrix<float, 3, 4> v = p_diff / dt;

  // collision point of edge a and b
  Eigen::Vector3f pa =
      (1.0f - para_a) * p_colli.col(0) + para_a * p_colli.col(1);
  Eigen::Vector3f pb =
      (1.0f - para_a) * p_colli.col(2) + para_b * p_colli.col(3);
  Eigen::Vector3f va = (1.0f - para_a) * v.col(0) + para_a * v.col(1);
  Eigen::Vector3f vb = (1.0f - para_a) * v.col(2) + para_b * v.col(3);

  Eigen::Vector3f n = (pb - pa).normalized();
  assert((n(0) != 0.0f || n(1) != 0.0f || n(2) != 0.0f) &&
         "collision normal is zero");

  // TODO: handle parallel edges at toi

  Eigen::Vector3f v_relative = va - vb;
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute impulse weight
  Eigen::Array4f para = {1.0f - para_a, para_a, 1.0f - para_b, para_b};
  Eigen::Array4f inv_mass;
  inv_mass(Eigen::seqN(0, 2)) = ma.inv_mass(Eigen::seqN(0, 2));
  inv_mass(Eigen::seqN(2, 2)) = mb.inv_mass(Eigen::seqN(0, 2));

  float denom = (para.square() * inv_mass).sum();
  Eigen::Vector4f weight = para.array() * inv_mass.array() / denom;
  weight(0) *= -1.0f;
  weight(1) *= -1.0f;

  // compute velocity after collision
  Eigen::Vector3f v_diff =
      (2.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  v += v_diff * weight.transpose();

  Collision collision;
  collision.type = CollisionType::EdgeEdge;
  collision.toi = toi;
  collision.offset(Eigen::seqN(0, 2)) =
      oa.solver_offset + 3 * ma.index(Eigen::seqN(0, 2)).array();
  collision.offset(Eigen::seqN(2, 2)) =
      ob.solver_offset + 3 * mb.index(Eigen::seqN(0, 2)).array();
  collision.reflection = p_colli + (1.0f - toi) * dt * v;

  SPDLOG_DEBUG("edge a v0 at t0: {}", position_t0.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 at t0: {}", position_t0.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 at t0: {}", position_t0.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 at t0: {}", position_t0.col(3).transpose());
  SPDLOG_DEBUG("edge a v0 at t1: {}", position_t1.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 at t1: {}", position_t1.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 at t1: {}", position_t1.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 at t1: {}", position_t1.col(3).transpose());
  SPDLOG_DEBUG("edge a v0 reflected: {}",
               collision.reflection.col(0).transpose());
  SPDLOG_DEBUG("edge a v1 reflected: {}",
               collision.reflection.col(1).transpose());
  SPDLOG_DEBUG("edge b v0 reflected: {}",
               collision.reflection.col(2).transpose());
  SPDLOG_DEBUG("edge b v1 reflected: {}",
               collision.reflection.col(3).transpose());
  SPDLOG_DEBUG(
      "EE: toi = [{}, {}], edge a para = [{}, {}], edge b para = [{}, {}], tol "
      "= {}",
      c->t(0), c->t(1), c->u(0), c->u(1), c->v(0), c->v(1), c->tolerance);

  return collision;
}

std::optional<Collision> narrow_phase(const ObjectCollider& oa,
                                      const MeshCollider& ma,
                                      const ObjectCollider& ob,
                                      const MeshCollider& mb, float dt,
                                      const Eigen::Array3f& scene_ee_err,
                                      const Eigen::Array3f& scene_vf_err) {
  // edge edge collision
  if (ma.type == MeshColliderType::Edge) {
    return edge_edge_collision(oa, ma, ob, mb, dt, scene_ee_err);
  }
  // point triangle collision: a is point and b is triangle
  else if (ma.type == MeshColliderType::Point) {
    return point_triangle_collision(oa, ma, ob, mb, dt, scene_vf_err);
  }
  // point triangle collision: a is triangle and b is point
  else {
    return point_triangle_collision(ob, mb, oa, ma, dt, scene_vf_err);
  }

  assert(false && "unreachable code path");
  return std::nullopt;
}

}  // namespace silk
