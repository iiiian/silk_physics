#pragma once

#include <Eigen/Core>
#include <optional>

namespace silk {

class CCDPoly {
  // CCD polynomial: ax^3 + bx^2 + cx + d = 0.
  float a_, b_, c_, d_;

  float tol_;
  float refine_it_;
  float eps_;

  std::optional<float> linear_ccd(float a, float b) const;
  std::optional<float> quadratic_ccd(float a, float b, float c) const;
  float eval(float x) const;
  float eval_derivative(float x) const;
  float refine_newton(float x) const;
  float forward_newton(float x) const;
  float backward_newton(float x) const;
  std::optional<float> cubic_ccd() const;

  CCDPoly() = default;

 public:
  // fail when poly completely degenerate. aka always coplanar case
  static std::optional<CCDPoly> try_make_ccd_poly(
      const Eigen::Matrix<float, 3, 4>& position_t0,
      const Eigen::Matrix<float, 3, 4>& position_t1, float tol, int refine_it,
      float eps);

  std::optional<float> solve() const;
};

}  // namespace silk
