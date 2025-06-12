#include <Eigen/Dense>
#include <cstdint>
#include <optional>

#include "common_types.hpp"

namespace eg = Eigen;

enum class CollisionConstrainType : uint8_t {
  PointPoint,
  PointEdge,
  PointTriangle,
  EdgeEdge
};

class CollisionConstrain {
  CollisionConstrainType type;
  uint32_t a, b, c, d;

  CollisionConstrain(CollisionConstrainType type, uint32_t a, uint32_t b,
                     uint32_t c, uint32_t d)
      : type(type), a(a), b(b), c(c), d(d) {}

 public:
  static CollisionConstrain make_point_point_constrain(uint32_t p1,
                                                       uint32_t p2) {
    return CollisionConstrain(CollisionConstrainType::PointPoint, p1, p1, 0, 0);
  }

  static CollisionConstrain make_point_edge_constrain(uint32_t p, uint32_t v1,
                                                      uint32_t v2) {
    return CollisionConstrain(CollisionConstrainType::PointEdge, p, v1, v2, 0);
  }

  static CollisionConstrain make_point_triangle_constrain(uint32_t p,
                                                          uint32_t v1,
                                                          uint32_t v2,
                                                          uint32_t v3) {
    return CollisionConstrain(CollisionConstrainType::PointTriangle, p, v1, v2,
                              v3);
  }

  static CollisionConstrain make_edge_edge_constrain(uint32_t e1v1,
                                                     uint32_t e1v2,
                                                     uint32_t e2v1,
                                                     uint32_t e2v2) {
    return CollisionConstrain(CollisionConstrainType::EdgeEdge, e1v1, e1v2,
                              e2v1, e2v2);
  }

  CollisionConstrainType get_type() const { return type; }
};

inline float compute_point_point_squared_distance(
    eg::Ref<const eg::Vector3f> p1, eg::Ref<const eg::Vector3f> p2) {
  return (p1 - p2).squaredNorm();
}

inline float compute_point_edge_squared_distance(
    eg::Ref<const eg::Vector3f> p, eg::Ref<const eg::Vector3f> v1,
    eg::Ref<const eg::Vector3f> v2) {
  return (v1 - p).cross(v2 - p).squaredNorm() / (v2 - v1).squaredNorm();
}

inline float compute_point_triangle_squared_distance(
    eg::Ref<const eg::Vector3f> p, eg::Ref<const eg::Vector3f> v1,
    eg::Ref<const eg::Vector3f> v2, eg::Ref<const eg::Vector3f> v3) {
  eg::Vector3f norm = (v2 - v1).cross(v3 - v1);
  double proj = (p - v1).dot(norm);
  return proj * proj / norm.squaredNorm();
}

std::optional<CollisionConstrain> compute_point_triangle_collision_constrain(
    uint32_t p_idx, uint32_t v1_idx, uint32_t v2_idx, uint32_t v3_idx,
    const RMatrixX3f& V, float squared_distance) {
  auto p = V.row(p_idx);
  auto v1 = V.row(v1_idx);
  auto v2 = V.row(v2_idx);
  auto v3 = V.row(v3_idx);

  eg::Matrix<float, 2, 3> param;
  eg::Matrix<float, 2, 3> basis;
  basis.row(0) = v2 - v1;
  basis.row(1) = v3 - v1;
  const eg::Vector3f norm = basis.row(0).cross(basis.row(1));

  basis.row(1) = basis.row(0).cross(norm);
  param.col(0) =
      (basis * basis.transpose()).ldlt().solve(basis * (p - v1).transpose());
  // point edge constrain, p - edge v1v2
  if (param(0, 0) > 0.0 && param(0, 0) < 1.0 && param(1, 0) >= 0.0) {
    if (compute_point_edge_squared_distance(p, v1, v2) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_edge_constrain(p_idx, v1_idx, v2_idx);
  }

  basis.row(0) = v3 - v2;
  basis.row(1) = basis.row(0).cross(norm);
  param.col(1) =
      (basis * basis.transpose()).ldlt().solve(basis * (p - v2).transpose());
  // point edge constrain, p - edge v2v3
  if (param(0, 1) > 0.0 && param(0, 1) < 1.0 && param(1, 1) >= 0.0) {
    if (compute_point_edge_squared_distance(p, v2, v3) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_edge_constrain(p_idx, v2_idx, v3_idx);
  }

  basis.row(0) = v1 - v3;
  basis.row(1) = basis.row(0).cross(norm);
  param.col(2) =
      (basis * basis.transpose()).ldlt().solve(basis * (p - v3).transpose());
  // point edge constrain, p - edge v3v1
  if (param(0, 2) > 0.0 && param(0, 2) < 1.0 && param(1, 2) >= 0.0) {
    if (compute_point_edge_squared_distance(p, v3, v1) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_edge_constrain(p_idx, v3_idx, v1_idx);
  }

  // point point constrain, p - v1
  if (param(0, 0) <= 0.0 && param(0, 2) >= 1.0) {
    if (compute_point_point_squared_distance(p, v1) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_point_constrain(p_idx, v1_idx);
  }

  // point point constrain, p - v2
  if (param(0, 1) <= 0.0 && param(0, 0) >= 1.0) {
    if (compute_point_point_squared_distance(p, v2) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_point_constrain(p_idx, v2_idx);
  }

  // point point constrain, p - v3
  if (param(0, 2) <= 0.0 && param(0, 1) >= 1.0) {
    if (compute_point_point_squared_distance(p, v3) > squared_distance) {
      return std::nullopt;
    }
    return CollisionConstrain::make_point_point_constrain(p_idx, v3_idx);
  }

  // point triangle constrain
  if (compute_point_triangle_squared_distance(p, v1, v2, v3) >
      squared_distance) {
    return std::nullopt;
  }
  return CollisionConstrain::make_point_triangle_constrain(p_idx, v1_idx,
                                                           v2_idx, v3_idx);
}
