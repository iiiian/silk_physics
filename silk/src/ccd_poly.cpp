#include "ccd_poly.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>

namespace silk {

std::optional<float> CCDPoly::linear_ccd(float a, float b) const {
  if (std::abs(a) < eps_) {
    return std::nullopt;
  }

  float tmp0 = -b / a;
  // TODO: tol
  if (tmp0 > 1 + eps_ || tmp0 < eps_) {
    return std::nullopt;
  }
  return tmp0;
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

float CCDPoly::refine_newton(float x) const {
  for (int i = 0; i < refine_it_; ++i) {
    x = x - eval(x) / eval_derivative(x);
  }
  return x;
}

float CCDPoly::forward_newton(float x) const {
  float x_next = x + tol_;
  float x_next_eval = eval(x_next);
  while (x_next_eval > 0) {
    x = x_next - x_next_eval / eval_derivative(x_next);
    x_next = x + tol_;
    x_next_eval = eval(x_next);

    assert((x > 0.0f && x < 1.0f));
  }

  // if (x < tol_) {
  //   return refine_newton(x);
  // }

  return x;
}

float CCDPoly::backward_newton(float x) const {
  float x_next = x - tol_;
  float x_next_eval = eval(x_next);
  while (x_next_eval < 0) {
    x = x_next - x_next_eval / eval_derivative(x_next);
    x_next = x - tol_;
    x_next_eval = eval(x_next);

    assert((x > 0.0f && x < 1.0f));
  }

  if (x_next < tol_) {
    return refine_newton(x);
  }

  return x_next;
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
      return backward_newton(1.0f);
    }
    // collision between t0, mirror point
    else {
      return forward_newton(0.0f);
    }
  }

  float tmp1 = -b_ / (3 * a_);  // mirror point
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
      if (tmp1 > 1.0f) {
        return backward_newton(1.0f);
      } else {
        return forward_newton(tmp1);
      }
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
      return forward_newton(tmp1);
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
      if (tmp1 > 1.0f) {
        return backward_newton(1.0f);
      } else {
        return forward_newton(tmp1);
      }
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
    if (eval(tmp1) > 0) {
      return forward_newton(tmp1);
    } else {
      return backward_newton(tmp1);
    }
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
    return backward_newton(1.0f);
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
    return backward_newton(1.0f);
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
    return forward_newton(0.0f);
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
    return forward_newton(0.0f);
  }

  // t0 before e1, t1 after e2
  // potential collision between t0 e1 and e2 t1
  // test t0 e1 collision
  float eval_e1 = eval(e1);
  if (eval_e1 < eps_) {
    if (eval_e1 > -eps_) {
      return eval_e1;
    }
    return forward_newton(0.0f);
  }
  // no collision between t0 e1, test e2 t1
  if (eval_t1 > eps_) {
    return std::nullopt;
  }
  if (eval_t1 > -eps_) {
    return eval_t1;
  }
  return backward_newton(1.0f);
}

std::optional<CCDPoly> CCDPoly::try_make_ccd_poly(
    const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1, float tol, int refine_it,
    float eps) {
  // unpack position
  auto x00 = position_t0.col(0);  // vertex 0 at t0
  auto x10 = position_t0.col(1);  // vertex 1 at t0
  auto x20 = position_t0.col(2);  // vertex 2 at t0
  auto x30 = position_t0.col(3);  // vertex 3 at t0
  auto x01 = position_t1.col(0);  // vertex 0 at t1
  auto x11 = position_t1.col(1);  // vertex 1 at t1
  auto x21 = position_t1.col(2);  // vertex 2 at t1
  auto x31 = position_t1.col(3);  // vertex 3 at t1

  Eigen::Vector3f p10 = x10 - x00;
  Eigen::Vector3f v10 = (x11 - x01) - p10;
  Eigen::Vector3f p20 = x20 - x00;
  Eigen::Vector3f v20 = (x21 - x01) - p20;
  Eigen::Vector3f p30 = x30 - x00;
  Eigen::Vector3f v30 = (x31 - x01) - p30;

  Eigen::Vector3f v10cv20 = v10.cross(v20);
  Eigen::Vector3f p10cv20 = p10.cross(v20);
  Eigen::Vector3f v10cp20 = v10.cross(p20);
  Eigen::Vector3f p10cp20 = p10.cross(p20);

  float a = v10cv20.dot(v30);
  float b = p10cv20.dot(v30) + v10cp20.dot(v30) + v10cv20.dot(p30);
  float c = v10cp20.dot(p30) + p10cv20.dot(p30) + p10cp20.dot(v30);
  float d = p10cp20.dot(p30);

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
  poly.refine_it_ = refine_it;
  poly.eps_ = eps;

  return poly;
}

std::optional<float> CCDPoly::solve() const { return cubic_ccd(); }

}  // namespace silk
