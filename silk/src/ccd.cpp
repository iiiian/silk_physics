#include "ccd.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

#include "ccd_poly.hpp"

namespace silk {

std::optional<Collision> point_triangle_collision(
    float toi, const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config) {
  const CCDConfig& c = config;

  // point triangle position at potential collision
  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  auto x0 = p_colli.col(0);  // point position
  auto x1 = p_colli.col(1);  // triangle vertex 1 position
  auto x2 = p_colli.col(2);  // triangle vertex 2 position
  auto x3 = p_colli.col(3);  // triangle vertex 3 position

  Eigen::Vector3f x13 = x1 - x3;
  Eigen::Vector3f x23 = x2 - x3;
  Eigen::Vector3f x03 = x0 - x3;

  float x13dx13 = x13.squaredNorm();
  float x13dx23 = x13.dot(x23);
  float x23dx23 = x23.squaredNorm();
  float x13dx03 = x13.dot(x03);
  float x23dx03 = x23.dot(x03);
  float det = x13dx13 * x23dx23 - x13dx23 * x13dx23;

  // degenerate triangle, ignore
  float area2_eps = std::pow(c.eps * std::max(x13dx13, x23dx23), 2);
  if (det < area2_eps) {
    return std::nullopt;
  }

  // bary barycentric coordinate of point projection
  float b1 = (x23dx23 * x13dx03 - x13dx23 * x23dx03) / det;
  float b2 = (-x13dx23 * x13dx03 + x13dx13 * x23dx03) / det;

  // barycentric coordinate is outside of triangle
  if (b1 < -c.eps || b1 > 1 + c.eps || b2 < -c.eps || b2 > 1 + c.eps ||
      b1 + b2 > 1 + c.eps) {
    return std::nullopt;
  }

  Eigen::Vector3f proj = b1 * x1 + b2 * x2 + (1.0f - b1 - b2) * x3;
  // collision normal, point to triangle
  Eigen::Vector3f n = proj - x0;
  float dist2 = n.squaredNorm();
  if (dist2 > c.h * c.h) {
    return std::nullopt;
  }
  n /= std::sqrt(dist2);

  // compute relative velocity at impact
  Eigen::Matrix<float, 3, 4> v = p_diff / c.dt;
  Eigen::Vector3f proj_t0 = b1 * position_t0.col(1) + b2 * position_t0.col(2) +
                            (1.0f - b1 - b2) * position_t0.col(3);
  Eigen::Vector3f v_proj = (proj - proj_t0) / (toi * c.dt);
  Eigen::Vector3f v_relative = v.col(0) - v_proj;
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;

  // compute reflection
  Eigen::Vector4f norm_weight = weight.array() / weight.sum();
  norm_weight(0) *= -1.0f;
  Eigen::Vector3f v_diff =
      (1.0f - c.damping) * v_normal + (1.0f - c.friction) * v_parallel;
  v += v_diff * norm_weight.transpose();

  Collision collision;
  collision.type = CollisionType::PointTriangle;
  collision.toi = toi;
  collision.position = p_colli + (1.0f - toi) * c.dt * v;

  return collision;
}

// return normal if collide
std::optional<Collision> edge_edge_collision(
    float toi, const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config) {
  const CCDConfig& c = config;

  // edge edge position at potential collision
  Eigen::Matrix<float, 3, 4> p_diff = (position_t1 - position_t0);
  Eigen::Matrix<float, 3, 4> p_colli = position_t0 + toi * p_diff;
  auto x0 = p_colli.col(0);  // edge 1 vertex 1 position
  auto x1 = p_colli.col(1);  // edge 1 vertex 2 position
  auto x2 = p_colli.col(2);  // edge 2 vertex 1 position
  auto x3 = p_colli.col(3);  // edge 2 vertex 2 position

  Eigen::Vector3f x10 = x1 - x0;
  Eigen::Vector3f x32 = x3 - x2;
  Eigen::Vector3f x20 = x2 - x0;

  float x10dx10 = x10.squaredNorm();
  float x10dx32 = x10.dot(x32);
  float x32dx32 = x32.squaredNorm();
  float x10dx21 = x10.dot(x20);
  float x32dx20 = x32.dot(x20);
  float det = x10dx10 * x32dx32 - x10dx32 * x10dx32;

  // zero length edge, ignore
  if (x10dx10 < c.eps * c.eps || x32dx32 < c.eps * c.eps) {
    return std::nullopt;
  }

  float e10_para = 0;
  float e32_para = 0;

  // find edge parameter of collision point

  // parallel edge
  float area2_eps = pow(c.eps * std::max(x10dx10, x32dx32), 2);
  if (det < area2_eps) {
    // test x0 against edge x2 x3
    if (float para = x32.dot(-x20) / x32dx32; para > 0.0f && para < 1.0f) {
      Eigen::Vector3f x0c = x2 + para * x32;
      float dist2 = (x0 - x0c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }

      e10_para = 0.0f;
      e32_para = para;
    }
    // test x1 against edge x2 x3
    else if (float para = x32.dot(x1 - x2) / x32dx32;
             para > 0.0f && para < 1.0f) {
      Eigen::Vector3f x1c = x2 + para * x32;
      float dist2 = (x1 - x1c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }

      e10_para = 1.0f;
      e32_para = para;
    }
    // test x2 against edge x0 x1
    else if (float para = x10.dot(x20) / x10dx10; para > 0.0f && para < 1.0f) {
      Eigen::Vector3f x2c = x0 + para * x10;
      float dist2 = (x2 - x2c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }

      e10_para = para;
      e32_para = 0.0f;
    }
    // test x3 against edge x0 x1
    else if (float para = x10.dot(x3 - x0) / x10dx10;
             para > 0.0f && para < 1.0f) {
      Eigen::Vector3f x3c = x0 + para * x10;
      float dist2 = (x3 - x3c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }

      e10_para = para;
      e32_para = 1.0f;
    }
  }
  // non-parallel edge, compute the closest point between two infinite line
  // then clamp if necessary
  else {
    e10_para = (x32dx32 * x10dx21 - x10dx32 * x32dx20) / det;
    bool is_e10_para_clamped = false;
    if (e10_para < 0.0f) {
      e10_para = 0.0f;
      is_e10_para_clamped = true;
    } else if (e10_para > 1.0f) {
      e10_para = 1.0f;
      is_e10_para_clamped = true;
    }
    e32_para = (x10dx32 * x10dx21 - x10dx10 * x32dx20) / det;
    bool is_e32_para_clamped = false;
    if (e32_para < 0.0f) {
      e32_para = 0.0f;
      is_e32_para_clamped = true;
    } else if (e32_para > 1.0f) {
      e32_para = 1.0f;
      is_e32_para_clamped = true;
    }

    // both parameter of edge x0 x1 and edge x2 x3 are outside
    if (is_e10_para_clamped && is_e32_para_clamped) {
      // compute 2 possible collision point pairs then choose the closer one
      // candidate pair a
      Eigen::Vector3f e10c_a = x0 + e10_para * x10;
      float e32_para_a = std::clamp(x32.dot(e10c_a - x2) / x32dx32, 0.0f, 1.0f);
      Eigen::Vector3f e32c_a = x2 + e32_para_a * x32;

      // candidate pair b
      Eigen::Vector3f e32c_b = x2 + e32_para * x32;
      float e10_para_b = std::clamp(x10.dot(e32c_b - x0) / x10dx10, 0.0f, 1.0f);
      Eigen::Vector3f e10c_b = x0 + e10_para_b * x10;

      float dist2a = (e10c_a - e32c_a).squaredNorm();
      float dist2b = (e10c_b - e32c_b).squaredNorm();
      if (dist2a < dist2b) {
        if (dist2a > c.h * c.h) {
          return std::nullopt;
        }
        e32_para = e32_para_a;
      } else {
        if (dist2b > c.h * c.h) {
          return std::nullopt;
        }
        e10_para = e10_para_b;
      }
    }
    // parameter of edge x0 x1 is outside
    else if (is_e10_para_clamped) {
      Eigen::Vector3f e10c = x0 + e10_para * x10;
      e32_para = std::clamp(x32.dot(e10c - x2) / x32dx32, 0.0f, 1.0f);
      Eigen::Vector3f e32c = x2 + e32_para * x32;
      float dist2 = (e10c - e32c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }
    }
    // parameter of edge x2 x3 is outside
    else if (is_e32_para_clamped) {
      Eigen::Vector3f e32c = x2 + e32_para * x32;
      e10_para = std::clamp(x10.dot(e32c - x0) / x10dx10, 0.0f, 1.0f);
      Eigen::Vector3f e12c = x0 + e10_para * x10;
      float dist2 = (e12c - e32c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }
    }
    // both para are inside
    else {
      Eigen::Vector3f e10c = x0 + e10_para * x10;
      Eigen::Vector3f e32c = x2 + e32_para * x32;
      float dist2 = (e10c - e32c).squaredNorm();
      if (dist2 > c.h * c.h) {
        return std::nullopt;
      }
    }
  }

  // compute relative velocity at impact
  Eigen::Matrix<float, 3, 4> v = p_diff / c.dt;
  Eigen::Vector3f v_e10c = e10_para * v.col(0) + (1.0f - e10_para) * v.col(1);
  Eigen::Vector3f v_e32c = e32_para * v.col(2) + (1.0f - e32_para) * v.col(3);
  Eigen::Vector3f v_relative = v_e10c - v_e32c;

  // compute collision normal n
  Eigen::Vector3f n;
  // parallel edge
  if (det < area2_eps) {
    n = x10.cross(x10.cross(x20)).normalized();
  }
  // non parallel edge
  else {
    n = x10.cross(x32).normalized();
  }

  // compute velocity after reflection
  Eigen::Vector3f v_normal = n.dot(v_relative) * n;
  Eigen::Vector3f v_parallel = v_relative - v_normal;
  float tmp0 = (weight(0) * e10_para + weight(1) * (1.0f - e10_para) +
                weight(2) * e32_para + weight(3) * (1.0f - e32_para));
  if (tmp0 == 0.0f) {
    return std::nullopt;
  }

  // Eigen::Vector4f coeff = weight.array() / tmp0;
  Eigen::Vector4f coeff = Eigen::Vector4f::Identity();
  coeff(0) *= -1.0f;
  coeff(1) *= -1.0f;
  // Eigen::Vector3f v_diff =
  //     (2.0f - c.damping) * v_normal + (1.0f - c.friction) * v_parallel;
  Eigen::Vector3f v_diff = (2.0f - c.damping) * v_normal;

  v += v_diff * coeff.transpose();

  Collision collision;
  collision.type = CollisionType::EdgeEdge;
  collision.toi = toi;
  collision.position = p_colli + (1.0f - toi) * c.dt * v;

  return collision;
}

std::optional<Collision> point_triangle_ccd(
    const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config) {
  auto poly = CCDPoly::try_make_ccd_poly(position_t0, position_t1, config.tol,
                                         config.bisect_it, config.eps);
  if (!poly) {
    return std::nullopt;
  }

  auto toi = poly->solve();
  if (!toi) {
    return std::nullopt;
  }

  return point_triangle_collision(*toi, position_t0, position_t1, weight,
                                  config);
}

std::optional<Collision> edge_edge_ccd(
    const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config) {
  auto poly = CCDPoly::try_make_ccd_poly(position_t0, position_t1, config.tol,
                                         config.bisect_it, config.eps);
  if (!poly) {
    return std::nullopt;
  }

  auto toi = poly->solve();
  if (!toi) {
    return std::nullopt;
  }

  return edge_edge_collision(*toi, position_t0, position_t1, weight, config);
}

}  // namespace silk
