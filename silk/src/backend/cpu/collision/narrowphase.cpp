#include "backend/cpu/collision/narrowphase.hpp"

#include "backend/cpu/collision/ccd.hpp"
#include "backend/cpu/collision/dcd.hpp"
#include "common/compiler_builtin.hpp"

namespace silk::cpu {

namespace {

constexpr float DCD_GEOMETRY_EPS = 1e-6f;

Eigen::Matrix3f get_position(const ObjectCollider& object,
                             const MeshCollider& mesh, float time) {
  if (object.state_offset != -1) {
    // Dynamic colliders are refit to the fixed rolled-back state before DCD.
    return mesh.position_t1;
  }
  // Obstacle colliders retain the full frame trajectory.
  return mesh.position_t0 + time * (mesh.position_t1 - mesh.position_t0);
}

void get_trajectory(const ObjectCollider& oa, const MeshCollider& ma,
                    const ObjectCollider& ob, const MeshCollider& mb,
                    int count_a, float time_start,
                    Eigen::Matrix<float, 3, 4>& position_t0,
                    Eigen::Matrix<float, 3, 4>& position_t1) {
  auto gather = [time_start](const ObjectCollider& object,
                             const MeshCollider& mesh, int count, auto block_t0,
                             auto block_t1) {
    // Interpolate obstacle.
    if (object.state_offset == -1) {
      Eigen::Matrix3f delta = mesh.position_t1 - mesh.position_t0;
      block_t0 =
          mesh.position_t0.leftCols(count) + time_start * delta.leftCols(count);
      block_t1 = block_t0 + delta.leftCols(count);
    }
    // For physical obj, just get current simulation state.
    else {
      block_t0 = mesh.position_t0.leftCols(count);
      block_t1 = mesh.position_t1.leftCols(count);
    }
  };

  gather(oa, ma, count_a, position_t0.leftCols(count_a),
         position_t1.leftCols(count_a));
  int count_b = 4 - count_a;
  gather(ob, mb, count_b, position_t0.rightCols(count_b),
         position_t1.rightCols(count_b));
}

std::optional<float> point_triangle_ccd(
    const ObjectCollider& point_object, const MeshCollider& point,
    const ObjectCollider& triangle_object, const MeshCollider& triangle,
    float time_start, float max_time, float minimum_separation, float tolerance,
    int max_iter, const Eigen::Array3f& scene_vf_err) {
  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  get_trajectory(point_object, point, triangle_object, triangle, 1, time_start,
                 position_t0, position_t1);

  auto result = vertex_face_ccd(
      position_t0.col(0), position_t0.col(1), position_t0.col(2),
      position_t0.col(3), position_t1.col(0), position_t1.col(1),
      position_t1.col(2), position_t1.col(3), scene_vf_err, minimum_separation,
      tolerance, max_iter, true, max_time);
  if (!result) {
    return std::nullopt;
  }
  if (!result->use_small_ms) {
    return result->t(0);
  }
  if (result->small_ms_t(0) == 0.0f) {
    return std::nullopt;
  }
  return result->small_ms_t(0);
}

std::optional<float> edge_edge_ccd_query(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float time_start, float max_time,
    float minimum_separation, float tolerance, int max_iter,
    const Eigen::Array3f& scene_ee_err) {
  Eigen::Matrix<float, 3, 4> position_t0;
  Eigen::Matrix<float, 3, 4> position_t1;
  get_trajectory(oa, ma, ob, mb, 2, time_start, position_t0, position_t1);

  auto result =
      edge_edge_ccd(position_t0.col(0), position_t0.col(1), position_t0.col(2),
                    position_t0.col(3), position_t1.col(0), position_t1.col(1),
                    position_t1.col(2), position_t1.col(3), scene_ee_err,
                    minimum_separation, tolerance, max_iter, true, max_time);
  if (!result) {
    return std::nullopt;
  }
  if (!result->use_small_ms) {
    return result->t(0);
  }
  if (result->small_ms_t(0) == 0.0f) {
    return std::nullopt;
  }
  return result->small_ms_t(0);
}

Collision make_collision(const ObjectCollider& oa, const ObjectCollider& ob,
                         CollisionType type, const Eigen::Vector4i& index,
                         const Eigen::Vector4f& inv_mass,
                         float activation_distance, float stiffness,
                         const Eigen::Vector4f& weight,
                         const Eigen::Vector3f& correction) {
  Collision collision;
  collision.type = type;
  collision.entity_a = oa.entity;
  collision.entity_b = ob.entity;
  collision.state_offset_a = oa.state_offset;
  collision.state_offset_b = ob.state_offset;
  collision.index = index;
  collision.activation_distance = activation_distance;
  collision.stiffness = stiffness;
  collision.inv_mass = inv_mass;
  collision.correction = correction * weight.transpose();
  return collision;
}

std::optional<Collision> point_triangle_dcd(
    const ObjectCollider& pt_obj, const MeshCollider& point,
    const ObjectCollider& tri_obj, const MeshCollider& triangle, float time,
    float activation_distance, float stiffness) {
  Eigen::Matrix<float, 3, 4> pos;
  pos.col(0) = get_position(pt_obj, point, time).col(0);
  pos.rightCols(3) = get_position(tri_obj, triangle, time).leftCols(3);

  auto uv = exact_pt_uv(pos.col(0), pos.col(1), pos.col(2), pos.col(3),
                        DCD_GEOMETRY_EPS);
  if (!uv) {
    return std::nullopt;
  }
  float u = uv->first;
  float v = uv->second;
  Eigen::Vector3f triangle_point =
      eval_triangle_parameter(u, v, pos.col(1), pos.col(2), pos.col(3));
  Eigen::Vector3f displacement = triangle_point - pos.col(0);
  float distance = displacement.norm();
  if (distance >= activation_distance) {
    return std::nullopt;
  }

  if (distance == 0.0f) {
    return std::nullopt;
  }
  Eigen::Vector3f normal = displacement / distance;

  Eigen::Vector4f parameter{1.0f, 1.0f - u - v, u, v};
  Eigen::Vector4f inv_mass;
  inv_mass(0) = point.inv_mass(0);
  inv_mass.tail<3>() = triangle.inv_mass;
  float denominator = (parameter.array().square() * inv_mass.array()).sum();
  if (denominator == 0.0f) {
    return std::nullopt;
  }
  Eigen::Vector4f weight = parameter.array() * inv_mass.array() / denominator;
  weight(0) *= -1.0f;

  Eigen::Vector4i index;
  index(0) = point.index(0);
  index.tail<3>() = triangle.index;
  return make_collision(pt_obj, tri_obj, CollisionType::PointTriangle, index,
                        inv_mass, activation_distance, stiffness, weight,
                        (activation_distance - distance) * normal);
}

std::optional<Collision> edge_edge_dcd(const ObjectCollider& oa,
                                       const MeshCollider& ma,
                                       const ObjectCollider& ob,
                                       const MeshCollider& mb, float time,
                                       float activation_distance,
                                       float stiffness) {
  Eigen::Matrix<float, 3, 4> pos;
  pos.leftCols(2) = get_position(oa, ma, time).leftCols(2);
  pos.rightCols(2) = get_position(ob, mb, time).leftCols(2);

  auto uv = exact_ee_uv(pos.col(0), pos.col(1), pos.col(2), pos.col(3),
                        DCD_GEOMETRY_EPS);
  if (!uv) {
    return std::nullopt;
  }
  float u = uv->first;
  float v = uv->second;
  Eigen::Vector3f point_a = eval_edge_parameter(u, pos.col(0), pos.col(1));
  Eigen::Vector3f point_b = eval_edge_parameter(v, pos.col(2), pos.col(3));
  Eigen::Vector3f displacement = point_b - point_a;
  float distance = displacement.norm();
  if (distance >= activation_distance) {
    return std::nullopt;
  }
  if (distance == 0.0f) {
    return std::nullopt;
  }
  Eigen::Vector3f normal = displacement / distance;

  Eigen::Vector4f parameter{1.0f - u, u, 1.0f - v, v};
  Eigen::Vector4f inv_mass;
  inv_mass.head<2>() = ma.inv_mass.head<2>();
  inv_mass.tail<2>() = mb.inv_mass.head<2>();
  float denominator = (parameter.array().square() * inv_mass.array()).sum();
  if (denominator == 0.0f) {
    return std::nullopt;
  }
  Eigen::Vector4f weight = parameter.array() * inv_mass.array() / denominator;
  weight.head<2>() *= -1.0f;

  Eigen::Vector4i index;
  index.head<2>() = ma.index.head<2>();
  index.tail<2>() = mb.index.head<2>();
  return make_collision(oa, ob, CollisionType::EdgeEdge, index, inv_mass,
                        activation_distance, stiffness, weight,
                        (activation_distance - distance) * normal);
}

}  // namespace

std::optional<float> find_min_toi(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float time_start, float max_time,
    float minimum_separation, float tolerance, int max_iter,
    const Eigen::Array3f& scene_ee_err, const Eigen::Array3f& scene_vf_err) {
  if (ma.type == MeshColliderType::Edge) {
    return edge_edge_ccd_query(oa, ma, ob, mb, time_start, max_time,
                               minimum_separation, tolerance, max_iter,
                               scene_ee_err);
  }
  if (ma.type == MeshColliderType::Point) {
    return point_triangle_ccd(oa, ma, ob, mb, time_start, max_time,
                              minimum_separation, tolerance, max_iter,
                              scene_vf_err);
  }
  if (ma.type == MeshColliderType::Triangle) {
    return point_triangle_ccd(ob, mb, oa, ma, time_start, max_time,
                              minimum_separation, tolerance, max_iter,
                              scene_vf_err);
  }
  SILK_UNREACHABLE();
}

std::optional<Collision> find_active_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float time, float activation_distance,
    float stiffness) {
  if (ma.type == MeshColliderType::Edge) {
    return edge_edge_dcd(oa, ma, ob, mb, time, activation_distance, stiffness);
  }
  if (ma.type == MeshColliderType::Point) {
    return point_triangle_dcd(oa, ma, ob, mb, time, activation_distance,
                              stiffness);
  }
  if (ma.type == MeshColliderType::Triangle) {
    return point_triangle_dcd(ob, mb, oa, ma, time, activation_distance,
                              stiffness);
  }
  SILK_UNREACHABLE();
}

}  // namespace silk::cpu
