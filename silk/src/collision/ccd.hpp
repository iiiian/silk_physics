#pragma once

#include <Eigen/Core>
#include <optional>

namespace silk {

struct PointTriangleImpact {
  Eigen::Vector3f p;
  Eigen::Vector3f v1;
  Eigen::Vector3f v2;
  Eigen::Vector3f v3;
  Eigen::Vector3f normal;
  float toi;
};

struct EdgeEdgeImpact {
  Eigen::Vector3f v1;
  Eigen::Vector3f v2;
  Eigen::Vector3f v3;
  Eigen::Vector3f v4;
  Eigen::Vector3f normal;
  float toi;
};

std::optional<PointTriangleImpact> point_triangle_ccd(
    Eigen::Ref<const Eigen::Vector3f> p0, Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30, Eigen::Ref<const Eigen::Vector3f> p1,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31, float h, float tol, int refine_it,
    float eps);

std::optional<EdgeEdgeImpact> edge_edge_ccd(
    Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30,
    Eigen::Ref<const Eigen::Vector3f> v40,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31,
    Eigen::Ref<const Eigen::Vector3f> v41, float h, float tol, int refine_it,
    float eps);

}  // namespace silk
