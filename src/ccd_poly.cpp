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

  eg::Vector3f v21cv31 = v21.cross(v31);
  eg::Vector3f p21cv31 = p21.cross(v31);
  eg::Vector3f v21cp31 = v21.cross(p31);
  eg::Vector3f p21cp31 = p21.cross(p31);

  a = v21cv31.dot(v41);
  b = p21cv31.dot(v41) + v21cp31.dot(v41) + v21cv31.dot(p41);
  c = v21cp31.dot(p41) + p21cv31.dot(p41) + p21cp31.dot(v41);
  d = p21cp31.dot(p41);
}

inline float NormalizedCCDPolySolver::eval(float x) const {
  return x * (x * (x * a_ + b_) + c_) + d_;
}

inline float NormalizedCCDPolySolver::eval_derivative(float x) const {
  return x * (x * 3 * a_ + 2 * b_) + c_;
}

inline float NormalizedCCDPolySolver::newton(float x) const {
  for (int it = 0; it < max_iter_; ++it) {
    float x_next = x - eval(x) / eval_derivative(x);
    if (std::abs(x_next - x) < tol_) {
      return x_next;
    }
    x = x_next;
  }
  return x;
}

std::optional<float> NormalizedCCDPolySolver::linear_ccd() const {
  // since coplaner case has already been dealt before, this means object is
  // stationary
  if (std::abs(c_) < eps_) {
    return std::nullopt;
  }

  float tmp = -d_ / c_;
  // TODO: tol
  if (tmp > 1 + eps_ || tmp < eps_) {
    return std::nullopt;
  }
  return tmp;
}

std::optional<float> NormalizedCCDPolySolver::quadratic_ccd() const {
  if (std::abs(b_) < eps_) {
    return linear_ccd();
  }

  // b^2 - 4ac for quadratic
  float tmp0 = c_ * c_ - 4 * b_ * d_;

  // no root
  if (tmp0 < 0) {
    return std::nullopt;
  }

  // one root
  float tmp1 = std::sqrt(tmp0);
  float tmp2 = (-c_ - tmp1) / (2 * b_);
  // TODO: tol
  if (tmp2 > 0 && tmp2 < 1) {
    return tmp2;
  }
  float tmp3 = (-c_ + tmp1) / (2 * b_);
  if (tmp3 > 0 && tmp3 < 1) {
    return tmp3;
  }
  return std::nullopt;
}

std::optional<float> NormalizedCCDPolySolver::coplaner_linear_ccd() const {
  // complete coplaner motion, we are in big trouble
  if (std::abs(b_) < eps_) {
    return std::nullopt;
  }

  float tmp0 = -c_ / b_;
  if (tmp0 < eps_ || tmp0 > 1) {
    return std::nullopt;
  }
  return tmp0;
}
std::optional<float> NormalizedCCDPolySolver::coplaner_quadratic_ccd() const {
  if (std::abs(a_) < eps_) {
    return coplaner_linear_ccd();
  }

  // b^2 - 4ac for quadratic
  float tmp0 = b_ * b_ - 4 * a_ * c_;

  // no root
  if (tmp0 < 0) {
    return std::nullopt;
  }
  // one root
  float tmp2 = std::sqrt(tmp0);
  float tmp3 = (-b_ - tmp2) / (2 * a_);
  // test first root
  if (tmp3 < eps_ || tmp3 > 1 + eps_) {
    return std::nullopt;
  }
  return tmp3;
}

std::optional<float> NormalizedCCDPolySolver::cubic_ccd() {
  float scale = 1.0f / std::max(std::max(std::abs(a_), std::abs(b_)),
                                std::max(std::abs(c_), std::abs(d_)));
  a_ *= scale;
  b_ *= scale;
  c_ *= scale;
  d_ *= scale;

  if (std::abs(d_) < eps_) {
    return coplaner_quadratic_ccd();
  }

  // make sure eval(t0) > 0
  if (d_ < 0) {
    a_ = -a_;
    b_ = -b_;
    c_ = -c_;
    d_ = -d_;
  }

  if (std::abs(a_) < eps_) {
    return quadratic_ccd();
  }

  float tmp0 = b_ * b_ - 3 * a_ * c_;
  float eval_t1 = a_ + b_ + c_ + d_;
  // case 1, 2, 3, 4
  if (tmp0 < eps_) {
    // case 1, 3
    if (a_ > 0) {
      return std::nullopt;
    }

    // case 2, 4
    if (eval_t1 > eps_) {
      return std::nullopt;
    }
    if (eval_t1 > -eps_) {
      return eval_t1;
    }

    float eval_mirror = eval(-b_ / a_ / 3);  // mirror point
    if (std::abs(eval_mirror) < eps_) {
      return eval_mirror;
    }
    // collision between mirror point, t1
    if (eval_mirror > 0) {
      return newton(1);
    }
    // collision between t0, mirror point
    else {
      return newton(0);
    }
  }

  float tmp1 = -b_ / (3 * a_);
  float tmp2 = std::sqrt(tmp0) / (3 * a_);
  // root of first derivative
  float e1 = tmp1 - tmp2;
  float e2 = tmp1 + tmp2;

  // case 5
  if (a_ > 0) {
    // t0 after e2
    // no collision
    if (0 > e2) {
      return std::nullopt;
    }

    // t0 after e1 before e2, t1 before e2
    // potential collision between t0 t1
    if (0 > e1 && 1 < e2) {
      if (eval_t1 > eps_) {
        return std::nullopt;
      }
      if (eval_t1 > -eps_) {
        return eval_t1;
      }
      // start from mirror point
      return newton(tmp1);
    }

    // t0 after e1 before e2, t1 after e2
    // potential collision between t0, e2
    if (0 > e1 && 1 > e2) {
      float eval_e2 = eval(e2);
      if (eval_e2 > eps_) {
        return std::nullopt;
      }
      if (eval_e2 > -eps_) {
        return eval_e2;
      }
      // start from mirror point
      return newton(tmp1);
    }

    // t0 before e1, t1 before e1
    // no collision
    if (1 < e1) {
      return std::nullopt;
    }

    // t0 before e1, t1 after e1 before e2
    // potential collision between e1 t1
    if (1 < e2) {
      if (eval_t1 > eps_) {
        return std::nullopt;
      }
      if (eval_t1 > -eps_) {
        return eval_t1;
      }
      // start from mirror point
      return newton(tmp1);
    }

    // t0 before e1, t1 after e2
    // potential between e1 e2
    float eval_e2 = eval(e2);
    if (eval_e2 > eps_) {
      return std::nullopt;
    }
    if (eval_e2 > -eps_) {
      return eval_e2;
    }
    // start from mirror point
    return newton(tmp1);
  }

  // case 6

  // t0 after e2
  // potential collision between t0 t1
  if (0 > e2) {
    if (eval_t1 > eps_) {
      return std::nullopt;
    }
    if (eval_t1 > -eps_) {
      return eval_t1;
    }
    return newton(1);
  }

  // t0 after e1 before e2, t1 before e2
  // no collision
  if (0 > e1 && 1 < e2) {
    return std::nullopt;
  }

  // t0 after e1 before e2 , t1 after e2
  // potential collision between e2 t1
  if (0 > e1 && 1 > e2) {
    if (eval_t1 > eps_) {
      return std::nullopt;
    }
    if (eval_t1 > -eps_) {
      return eval_t1;
    }
    return newton(1);
  }

  // t0 before e1, t1 before e1
  // potential collision before t0 t1
  if (1 < e1) {
    if (eval_t1 > eps_) {
      return std::nullopt;
    }
    if (eval_t1 > -eps_) {
      return eval_t1;
    }
    return newton(0);
  }

  // t0 before e1, t1 after e1 before e2
  // potential collision t0 e1
  if (1 < e2) {
    float eval_e1 = eval(e1);
    if (eval_e1 > eps_) {
      return std::nullopt;
    }
    if (eval_e1 > -eps_) {
      return eval_e1;
    }
    return newton(0);
  }

  // t0 before e1, t1 after e2
  // potential collision between t0 e1 and e2 t1
  // test t0 e1 collision
  float eval_e1 = eval(e1);
  if (eval_e1 < eps_) {
    if (eval_e1 > -eps_) {
      return eval_e1;
    }
    return newton(0);
  }
  // no collision between t0 e1, test e2 t1
  if (eval_t1 > eps_) {
    return std::nullopt;
  }
  if (eval_t1 > -eps_) {
    return eval_t1;
  }
  return newton(1);
}

std::optional<float> NormalizedCCDPolySolver::solve(
    const NormalizedCCDPoly& poly, float tol, int max_iter, float eps) {
  a_ = poly.a;
  b_ = poly.b;
  c_ = poly.c;
  d_ = poly.d;
  tol_ = tol;
  max_iter_ = max_iter;
  eps_ = eps;

  return cubic_ccd();
}
