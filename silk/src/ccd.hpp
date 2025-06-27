#pragma once

#include <Eigen/Core>
#include <optional>

class CCDSolver {
  std::optional<float> coplaner_point_triangle_ccd(
      Eigen::Ref<const Eigen::Vector3f> p0,
      Eigen::Ref<const Eigen::Vector3f> v10,
      Eigen::Ref<const Eigen::Vector3f> v20,
      Eigen::Ref<const Eigen::Vector3f> v30,
      Eigen::Ref<const Eigen::Vector3f> p1,
      Eigen::Ref<const Eigen::Vector3f> v11,
      Eigen::Ref<const Eigen::Vector3f> v21,
      Eigen::Ref<const Eigen::Vector3f> v31) const;

  std::optional<float> coplaner_edge_edge_ccd(
      Eigen::Ref<const Eigen::Vector3f> v10,
      Eigen::Ref<const Eigen::Vector3f> v20,
      Eigen::Ref<const Eigen::Vector3f> v30,
      Eigen::Ref<const Eigen::Vector3f> v40,
      Eigen::Ref<const Eigen::Vector3f> v11,
      Eigen::Ref<const Eigen::Vector3f> v21,
      Eigen::Ref<const Eigen::Vector3f> v31,
      Eigen::Ref<const Eigen::Vector3f> v41) const;

 public:
  float eps;
  float h;
  float tol;
  int max_iter;

  bool point_triangle_ccd(Eigen::Ref<const Eigen::Vector3f> p0,
                          Eigen::Ref<const Eigen::Vector3f> v10,
                          Eigen::Ref<const Eigen::Vector3f> v20,
                          Eigen::Ref<const Eigen::Vector3f> v30,
                          Eigen::Ref<const Eigen::Vector3f> p1,
                          Eigen::Ref<const Eigen::Vector3f> v11,
                          Eigen::Ref<const Eigen::Vector3f> v21,
                          Eigen::Ref<const Eigen::Vector3f> v31, float t0,
                          float t1);

  bool edge_edge_ccd(Eigen::Ref<const Eigen::Vector3f> v10,
                     Eigen::Ref<const Eigen::Vector3f> v20,
                     Eigen::Ref<const Eigen::Vector3f> v30,
                     Eigen::Ref<const Eigen::Vector3f> v40,
                     Eigen::Ref<const Eigen::Vector3f> v11,
                     Eigen::Ref<const Eigen::Vector3f> v21,
                     Eigen::Ref<const Eigen::Vector3f> v31,
                     Eigen::Ref<const Eigen::Vector3f> v41, float t0, float t1);
};
