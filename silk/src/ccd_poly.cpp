#include "ccd_poly.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <iostream>

namespace silk {

std::optional<float> CCDPoly::linear_ccd(float a, float b) const {
  if (std::abs(a) < eps_) {
    return std::nullopt;
  }

  float tmp = -b / a;
  // TODO: tol
  if (tmp > 1 + eps_ || tmp < eps_) {
    return std::nullopt;
  }
  return tmp;
}

std::optional<float> CCDPoly::quadratic_ccd(float a, float b, float c) const {
  if (std::abs(a) < eps_) {
    return linear_ccd(b, c);
  }

  float tmp0 = b * b - 4 * a * c;

  // no root
  if (tmp0 < 0) {
    return std::nullopt;
  }

  // one root
  float tmp1 = std::sqrt(tmp0);
  float tmp2 = (-b - tmp1) / (2 * a);
  // TODO: tol
  if (tmp2 > 0 && tmp2 < 1) {
    return tmp2;
  }
  float tmp3 = (-b + tmp1) / (2 * a);
  if (tmp3 > 0 && tmp3 < 1) {
    return tmp3;
  }
  return std::nullopt;
}

float CCDPoly::eval(float x) const { return x * (x * (x * a_ + b_) + c_) + d_; }

float CCDPoly::eval_derivative(float x) const {
  return x * (x * 3 * a_ + 2 * b_) + c_;
}

float CCDPoly::newton(float x) const {
  for (int it = 0; it < max_iter_; ++it) {
    float x_next = x - eval(x) / eval_derivative(x);
    if (std::abs(x_next - x) < tol_) {
      // std::cout << "newton it: " << it + 1 << "\n";
      return x_next;
    }
    x = x_next;
  }
  // std::cout << "newton it: " << max_iter_ << "\n";
  return x;
}

std::optional<float> CCDPoly::cubic_ccd() const {
  if (std::abs(d_) < eps_) {
    return quadratic_ccd(a_, b_, c_);
  }

  if (std::abs(a_) < eps_) {
    return quadratic_ccd(b_, c_, d_);
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

std::optional<CCDPoly> CCDPoly::try_make_ccd_poly(
    Eigen::Ref<const Eigen::Vector3f> x10,
    Eigen::Ref<const Eigen::Vector3f> x20,
    Eigen::Ref<const Eigen::Vector3f> x30,
    Eigen::Ref<const Eigen::Vector3f> x40,
    Eigen::Ref<const Eigen::Vector3f> x11,
    Eigen::Ref<const Eigen::Vector3f> x21,
    Eigen::Ref<const Eigen::Vector3f> x31,
    Eigen::Ref<const Eigen::Vector3f> x41, float tol, int max_iter, float eps) {
  Eigen::Vector3f p21 = x20 - x10;
  Eigen::Vector3f v21 = (x21 - x11) - p21;
  Eigen::Vector3f p31 = x30 - x10;
  Eigen::Vector3f v31 = (x31 - x11) - p31;
  Eigen::Vector3f p41 = x40 - x10;
  Eigen::Vector3f v41 = (x41 - x11) - p41;

  Eigen::Vector3f v21cv31 = v21.cross(v31);
  Eigen::Vector3f p21cv31 = p21.cross(v31);
  Eigen::Vector3f v21cp31 = v21.cross(p31);
  Eigen::Vector3f p21cp31 = p21.cross(p31);

  float a = v21cv31.dot(v41);
  float b = p21cv31.dot(v41) + v21cp31.dot(v41) + v21cv31.dot(p41);
  float c = v21cp31.dot(p41) + p21cv31.dot(p41) + p21cp31.dot(v41);
  float d = p21cp31.dot(p41);

  float max_coeff =
      std::max({std::abs(a), std::abs(b), std::abs(c), std::abs(d)});
  // Big trouble, complete coplaner ccd
  if (std::abs(max_coeff) < eps) {
    return std::nullopt;
  }

  float scale = 1 / max_coeff;
  a *= scale;
  b *= scale;
  c *= scale;
  d *= scale;

  // make sure eval(t0) > 0
  if (d < 0) {
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }

  CCDPoly poly;
  poly.a_ = a;
  poly.b_ = b;
  poly.c_ = c;
  poly.d_ = d;
  poly.tol_ = tol;
  poly.max_iter_ = max_iter;
  poly.eps_ = eps;

  return poly;
}

std::optional<float> CCDPoly::solve() const { return cubic_ccd(); }

}  // namespace silk
