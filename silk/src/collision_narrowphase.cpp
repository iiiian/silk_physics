#include "collision_narrowphase.hpp"

#include <cassert>
#include <tight_inclusion/ccd.hpp>
#include <tight_inclusion/interval_root_finder.hpp>

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
  float toi;
  float bary_a;
  float bary_b;
  float tol;
  bool is_colliding = ticcd::vertexFaceCCD(
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
      toi,                 // output parameter: time of impact
      bary_a,  // output parameter: barycentric coordinate of triangle
      bary_b,  // output parameter: barycentric coordinate of triangle
      1e-6f,   // ticcd solving precision. set to 1e-6f as recommended in README
      1.0f,    // max time, we use normalized time interval [0, 1]
      -1,      // max ticcd iteration, set as -1 to disable
      tol,     // ticcd solving precision if terminated by max iter cap.
      ticcd::DEFAULT_NO_ZERO_TOI,  // no toi refinement if toi = 0
      ticcd::CCDRootFindingMethod::BREADTH_FIRST_SEARCH);

  if (!is_colliding) {
    return std::nullopt;
  }

  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  Eigen::Matrix<float, 3, 4> v = p_diff / dt;

  // collision point of triangle
  Eigen::Vector3f pt = (1.0f - bary_a - bary_b) * p_colli.col(1) +
                       bary_a * p_colli.col(2) + bary_b * p_colli.col(3);
  Eigen::Vector3f v_pt = (1.0f - bary_a - bary_b) * v.col(1) +
                         bary_a * v.col(2) + bary_b * v.col(3);

  Eigen::Vector3f n = (pt - p_colli.col(0)).normalized();
  assert((n(0) != 0.0f || n(1) != 0.0f || n(2) != 0.0f) &&
         "collision normal is zero");

  Eigen::Vector3f v_relative = v.col(0) - v_pt;
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  Eigen::Vector4f impulse_weight;
  impulse_weight(Eigen::seqN(0, 1)) = ma.inv_mass(Eigen::seqN(0, 1));
  impulse_weight(Eigen::seqN(1, 3)) = mb.inv_mass(Eigen::seqN(0, 3));
  // normalize weights
  float weight_sum = impulse_weight.sum();
  impulse_weight /= weight_sum;
  // weight 0 is point, which will reflects in opposite direction
  impulse_weight(0) *= -1.0f;

  Eigen::Vector3f v_diff =
      (1.0f - damping) * v_normal + (1.0f - friction) * v_parallel;
  v += v_diff * impulse_weight.transpose();

  Collision collision;
  collision.type = CollisionType::PointTriangle;
  collision.toi = toi;
  collision.reflection = p_colli + (1.0f - toi) * dt * v;

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
  float toi;
  float para_a;
  float para_b;
  float tol;
  bool is_colliding = ticcd::vertexFaceCCD(
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
      toi,                 // output parameter: time of impact
      para_a,              // output parameter: edge a parameter
      para_b,              // output parameter: edge b parameter
      1e-6f,  // ticcd solving precision. set to 1e-6f as recommended in README
      1.0f,   // max time, we use normalized time interval [0, 1]
      -1,     // max ticcd iteration, set as -1 to disable
      tol,    // ticcd solving precision if terminated by max iter cap.
      ticcd::DEFAULT_NO_ZERO_TOI,  // no toi refinement if toi = 0
      ticcd::CCDRootFindingMethod::BREADTH_FIRST_SEARCH);

  if (!is_colliding) {
    return std::nullopt;
  }

  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  Eigen::Matrix<float, 3, 4> v = p_diff / dt;

  // collision point of edge a and b
  Eigen::Vector3f pa =
      para_a * p_colli.col(0) + (1.0f - para_a) * p_colli.col(1);
  Eigen::Vector3f pb =
      para_a * p_colli.col(2) + (1.0f - para_b) * p_colli.col(3);
  Eigen::Vector3f va = para_a * v.col(0) + (1.0f - para_a) * v.col(1);
  Eigen::Vector3f vb = para_a * v.col(2) + (1.0f - para_b) * v.col(3);

  Eigen::Vector3f n = (pb - pa).normalized();
  assert((n(0) != 0.0f || n(1) != 0.0f || n(2) != 0.0f) &&
         "collision normal is zero");

  // TODO: handle parallel edges at toi

  Eigen::Vector3f v_relative = va - vb;
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute impulse weight
  Eigen::Array4f para = {1.0f - float(para_a), float(para_a),
                         1.0f - float(para_b), float(para_b)};
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
  collision.reflection = p_colli + (1.0f - toi) * dt * v;

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
