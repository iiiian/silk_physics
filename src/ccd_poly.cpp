#include "ccd_poly.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>

namespace eg = Eigen;

NormalizedCCDPoly::NormalizedCCDPoly(
    eg::Ref<const eg::Vector3f> x10, eg::Ref<const eg::Vector3f> x20,
    eg::Ref<const eg::Vector3f> x30, eg::Ref<const eg::Vector3f> x40,
    eg::Ref<const eg::Vector3f> x11, eg::Ref<const eg::Vector3f> x21,
    eg::Ref<const eg::Vector3f> x31, eg::Ref<const eg::Vector3f> x41) {
  eg::Vector3f p21 = x20 - x10;
  eg::Vector3f v21 = (x21 - x11) - p21;
  eg::Vector3f p31 = x30 - x10;
  eg::Vector3f v31 = (x31 - x11) - p31;
  eg::Vector3f p41 = x40 - x10;
  eg::Vector3f v41 = (x41 - x11) - p41;

  a = v21.cross(v31).dot(v41);
  b = x21.cross(v31).dot(v41) + v21.cross(x31).dot(v41) +
      v21.cross(v31).dot(x41);
  c = v21.cross(x31).dot(x41) + x21.cross(v31).dot(x41) +
      x21.cross(x31).dot(v41);
  d = x21.cross(x31).dot(x41);
}

inline float CCDPolynomialSolver::eval(float x) const {
  return x * (x * (x * a_ + b_) + c_) + d_;
}

inline float CCDPolynomialSolver::eval_derivative(float x) const {
  return x * (x * 3 * a_ + 2 * b_) + c_;
}

inline float CCDPolynomialSolver::newton(float x) const {
  for (int it = 0; it < max_iter_; ++it) {
    float x_next = x - eval(x) / eval_derivative(x);
    if (std::abs(x_next - x) < tol_) {
      return x_next;
    }
    x = x_next;
  }
  return x;
}

inline CCDPolySolution CCDPolynomialSolver::try_clamp(float x) const {
  if (x < t0_) {
    if (x < t0_ - tol_) {
      return CCDPolySolution();
    }
    return CCDPolySolution(t0_);
  }

  if (x > t1_) {
    if (x > t1_ + tol_) {
      return CCDPolySolution();
    }
    return CCDPolySolution(t1_);
  }

  return CCDPolySolution(x);
}

CCDPolySolution CCDPolynomialSolver::linear_ccd() const {
  // if not even linear ccd exists, its likely the object are stationary.
  // hence no collision.
  if (std::abs(c_) < eps_) {
    return CCDPolySolution();
  }

  return try_clamp(-d_ / c_);
}

CCDPolySolution CCDPolynomialSolver::quadratic_ccd() const {
  if (std::abs(b_) < eps_) {
    return linear_ccd();
  }

  // b^2 - 4ac for quadratic
  float p0 = c_ * c_ - 4 * b_ * d_;

  if (p0 < -eps_) {
    return CCDPolySolution();
  }
  if (p0 < eps_) {
    return try_clamp(-c_ / (2 * b_));
  }
  return try_clamp((-c_ - std::sqrt(p0)) / (2 * b_));
}

CCDPolySolution CCDPolynomialSolver::cubic_ccd() const {
  if (std::abs(a_) < eps_) {
    return quadratic_ccd();
  }

  // b^2 - 4ac for first derivative
  float p0 = 4 * b_ * b_ - 12 * a_ * c_;
  // case 1, 2, 3, 4
  if (p0 < eps_) {
    // case 1, 3
    if (a_ > 0) {
      return CCDPolySolution();
    }

    // case 2, 4
    float p1 = eval(t1_);
    if (p1 > eps_) {
      return CCDPolySolution();
    } else if (p1 > -eps_) {
      return CCDPolySolution(p1);
    }

    float p2 = eval(-b_ / a_ / 3);  // mirror point
    // collision between mirror point, t1
    if (p2 > 0) {
      return CCDPolySolution(newton(t1_));
    }
    // collision between t0, mirror point
    return CCDPolySolution(newton(t0_));
  }

  float p3 = -b_ / (3 * a_);
  float p4 = std::sqrt(p0) / (3 * a_);
  // root of first derivative
  float p5 = p3 - p4;
  float p6 = p3 + p4;

  // case 5
  if (a_ > 0) {
    if (t0_ > p6 || t1_ < p5 || eval(p6) > eps_) {
      return CCDPolySolution();
    }

    // potential collision between p5, p6
    float p7 = -b_ / a_ / 3;  // mirror point
    return try_clamp(newton(p7));
  }

  // case 6

  // collision after p6
  if (t0_ > p5) {
    if (eval(t1_) > eps_) {
      return CCDPolySolution();
    }
    return try_clamp(newton(t1_));
  }

  // collision before p5
  if (t1_ < p6) {
    if (eval(std::min(t1_, p6)) > eps_) {
      return CCDPolySolution();
    }
    return try_clamp(newton(t0_));
  }

  // 2 potential collisions
  // one potential collision before p5
  CCDPolySolution result;
  float p8 = eval(p5);
  if (p8 < eps_) {
    result.n++;
    result.t1 = (p8 > -eps_) ? p5 : newton(t0_);
  }
  // one potential collision after p6
  float p9 = eval(t1_);
  if (p9 < eps_ && eval(p6) > 0) {
    result.n++;
    result.t2 = (p9 > -eps_) ? t1_ : newton(t1_);
  }
  return result;
}

CCDPolySolution CCDPolynomialSolver::solve(const NormalizedCCDPoly& poly,
                                           float tol, int max_iter, float eps) {
  a_ = poly.a;
  b_ = poly.b;
  c_ = poly.c;
  d_ = poly.d;
  t0_ = 0;
  t1_ = 1;
  tol_ = tol;
  max_iter_ = max_iter;
  eps_ = eps;

  return cubic_ccd();
}

CCDPolySolution CCDPolynomialSolver::solve(float a, float b, float c, float d,
                                           float t0, float t1, float tol,
                                           int max_iter, float eps) {
  assert((t1_ > t0_));

  a_ = a;
  b_ = b;
  c_ = c;
  d_ = d;
  t0_ = t0;
  t1_ = t1;
  tol_ = tol;
  max_iter_ = max_iter;
  eps_ = eps;

  return cubic_ccd();
}
