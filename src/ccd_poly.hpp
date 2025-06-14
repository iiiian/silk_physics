#pragma once

#include <Eigen/Core>
#include <optional>

struct NormalizedCCDPoly {
  float a, b, c, d;

  NormalizedCCDPoly(Eigen::Ref<const Eigen::Vector3f> x10,
                    Eigen::Ref<const Eigen::Vector3f> x20,
                    Eigen::Ref<const Eigen::Vector3f> x30,
                    Eigen::Ref<const Eigen::Vector3f> x40,
                    Eigen::Ref<const Eigen::Vector3f> x11,
                    Eigen::Ref<const Eigen::Vector3f> x21,
                    Eigen::Ref<const Eigen::Vector3f> x31,
                    Eigen::Ref<const Eigen::Vector3f> x41);
};

class NormalizedCCDPolySolver {
  // ax^3 + bx^2 + cx + d
  float a_, b_, c_, d_;

  float tol_;
  int max_iter_;
  float eps_;

  inline float eval(float x) const;
  inline float eval_derivative(float x) const;
  inline float newton(float x) const;
  std::optional<float> linear_ccd() const;
  std::optional<float> quadratic_ccd() const;
  std::optional<float> coplaner_linear_ccd() const;
  std::optional<float> coplaner_quadratic_ccd() const;
  std::optional<float> cubic_ccd();

 public:
  std::optional<float> solve(const NormalizedCCDPoly& poly, float tol,
                             int max_iter, float eps);
};
