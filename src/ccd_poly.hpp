#pragma once

#include <Eigen/Core>
#include <optional>

class CCDPoly {
  float a_, b_, c_, d_;
  float eps_;
  float tol_;
  float max_iter_;

  std::optional<float> linear_ccd(float a, float b) const;
  std::optional<float> quadratic_ccd(float a, float b, float c) const;
  inline float eval(float x) const;
  inline float eval_derivative(float x) const;
  inline float newton(float x) const;
  std::optional<float> cubic_ccd();

 public:
  CCDPoly(Eigen::Ref<const Eigen::Vector3f> x10,
          Eigen::Ref<const Eigen::Vector3f> x20,
          Eigen::Ref<const Eigen::Vector3f> x30,
          Eigen::Ref<const Eigen::Vector3f> x40,
          Eigen::Ref<const Eigen::Vector3f> x11,
          Eigen::Ref<const Eigen::Vector3f> x21,
          Eigen::Ref<const Eigen::Vector3f> x31,
          Eigen::Ref<const Eigen::Vector3f> x41);

  std::optional<float> solve(float tol, int max_iter, float eps);
};
