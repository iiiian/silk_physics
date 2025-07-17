#include "ccd.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "ccd_poly.hpp"

namespace silk {

// return normal if collide
std::optional<Eigen::Vector3f> point_triangle_collision(
    Eigen::Ref<const Eigen::Vector3f> p, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2, Eigen::Ref<const Eigen::Vector3f> v3,
    float h, float eps) {
  Eigen::Vector3f v13 = v1 - v3;
  Eigen::Vector3f v23 = v2 - v3;
  Eigen::Vector3f vp3 = p - v3;

  float v13dv13 = v13.squaredNorm();
  float v13dv23 = v13.dot(v23);
  float v23dv23 = v23.squaredNorm();
  float v13dvp3 = v13.dot(vp3);
  float v23dvp3 = v23.dot(vp3);
  float det = v13dv13 * v23dv23 - v13dv23 * v13dv23;

  // degenerate triangle
  // TODO: warn about degenerate triangle
  float area2_eps = std::pow(eps * std::max(v13dv13, v23dv23), 2);
  if (det < area2_eps) {
    return std::nullopt;
  }

  float b1 = (v23dv23 * v13dvp3 - v13dv23 * v23dvp3) / det;
  float b2 = (-v13dv23 * v13dvp3 + v13dv13 * v23dvp3) / det;

  // barycentric coordinate is outside of triangle
  if (b1 < -eps || b1 > 1 + eps || b2 < -eps || b2 > 1 + eps ||
      b1 + b2 > 1 + eps) {
    return std::nullopt;
  }

  Eigen::Vector3f proj = b1 * v1 + b2 * v2 + (1 - b1 - b2) * v3;
  Eigen::Vector3f n = p - proj;
  float dist2 = n.squaredNorm();
  if (dist2 > h * h) {
    return std::nullopt;
  }
  return n / std::sqrt(dist2);
}

// return normal if collide
std::optional<Eigen::Vector3f> edge_edge_collision(
    Eigen::Ref<const Eigen::Vector3f> v1, Eigen::Ref<const Eigen::Vector3f> v2,
    Eigen::Ref<const Eigen::Vector3f> v3, Eigen::Ref<const Eigen::Vector3f> v4,
    float h, float eps) {
  Eigen::Vector3f v21 = v2 - v1;
  Eigen::Vector3f v43 = v4 - v3;
  Eigen::Vector3f v31 = v3 - v1;

  float v21dv21 = v21.squaredNorm();
  float v21dv43 = v21.dot(v43);
  float v43dv43 = v43.squaredNorm();
  float v21dv31 = v21.dot(v31);
  float v43dv31 = v43.dot(v31);
  float det = v21dv21 * v43dv43 - v21dv43 * v21dv43;

  // zero length edge
  // TODO: warn zero length edge
  if (v21dv21 < eps || v43dv43 < eps) {
    return std::nullopt;
  }

  // parallel edge
  float area2_eps = pow(eps * std::max(v21dv21, v43dv43), 2);
  if (det < area2_eps) {
    float v1cp = v43.dot(-v31) / v43dv43;
    if (v1cp > 0.0f && v1cp < 1.0f) {
      Eigen::Vector3f v1c = v3 + v1cp * v43;
      Eigen::Vector3f n = v1 - v1c;
      float dist2 = n.squaredNorm();
      if (dist2 > h * h) {
        return std::nullopt;
      }
      return n / std::sqrt(dist2);
    }

    float v2cp = v43.dot(v2 - v3) / v43dv43;
    if (v2cp > 0.0f && v2cp < 1.0f) {
      Eigen::Vector3f v2c = v3 + v2cp * v43;
      Eigen::Vector3f n = v2 - v2c;
      float dist2 = n.squaredNorm();
      if (dist2 > h * h) {
        return std::nullopt;
      }
      return n / std::sqrt(dist2);
    }

    float v3cp = v21.dot(v31) / v21dv21;
    if (v3cp > 0.0f && v3cp < 1.0f) {
      Eigen::Vector3f v3c = v1 + v3cp * v21;
      Eigen::Vector3f n = -v3 + v3c;
      float dist2 = n.squaredNorm();
      if (dist2 > h * h) {
        return std::nullopt;
      }
      return n / std::sqrt(dist2);
    }

    float v4cp = v21.dot(v4 - v1) / v21dv21;
    if (v4cp > 0.0f && v4cp < 1.0f) {
      Eigen::Vector3f v4c = v1 + v4cp * v21;
      Eigen::Vector3f n = -v4 + v4c;
      float dist2 = n.squaredNorm();
      if (dist2 > h * h) {
        return std::nullopt;
      }
      return n / std::sqrt(dist2);
    }
  }

  // non parallel edge, compute the closest point between two infinite line
  // then clamp if necessary
  auto try_clamp = [](float val) -> std::pair<bool, float> {
    if (val < 0) {
      return {true, 0};
    }
    if (val > 1) {
      return {true, 1};
    }
    return {false, val};
  };
  auto [is_e12cp_clamped, e12cp] =
      try_clamp((v43dv43 * v21dv31 - v21dv43 * v43dv31) / det);
  auto [is_e34cp_clamped, e34cp] =
      try_clamp((v21dv43 * v21dv31 - v21dv21 * v43dv31) / det);

  // both parameter of edge v1 v2 and edge v3 v4 are outside
  if (is_e12cp_clamped && is_e34cp_clamped) {
    // compute 2 possible collision point pairs then choose the closer one
    // candidate pair a
    Eigen::Vector3f e12c_a = v1 + e12cp * v21;
    float e34cp_a = std::clamp(v43.dot(e12c_a - v3) / v43dv43, 0.0f, 1.0f);
    Eigen::Vector3f e34c_a = v3 + e34cp_a * v43;

    // candidate pair b
    Eigen::Vector3f e34c_b = v3 + e34cp * v43;
    float e12cp_b = std::clamp(v21.dot(e34c_b - v1) / v21dv21, 0.0f, 1.0f);
    Eigen::Vector3f e12c_b = v1 + e12cp_b * v21;

    Eigen::Vector3f na = e12c_a - e34c_a;
    Eigen::Vector3f nb = e12c_b - e34c_b;
    float dist2a = na.squaredNorm();
    float dist2b = nb.squaredNorm();
    if (dist2a < dist2b) {
      if (dist2a > h * h) {
        return std::nullopt;
      }
      return na / std::sqrt(dist2a);
    } else {
      if (dist2b > h * h) {
        return std::nullopt;
      }
      return nb / std::sqrt(dist2b);
    }
  }

  // parameter of edge v1 v2 is outside
  if (is_e12cp_clamped) {
    Eigen::Vector3f e12c = v1 + e12cp * v21;
    float e34cp = std::clamp(v43.dot(e12c - v3) / v43dv43, 0.0f, 1.0f);
    Eigen::Vector3f e34c = v3 + e34cp * v43;
    Eigen::Vector3f n = e12c - e34c;
    float dist2 = n.squaredNorm();
    if (dist2 > h * h) {
      return std::nullopt;
    }
    return n / std::sqrt(dist2);
  }

  // parameter of edge v3 v4 is outside
  if (is_e34cp_clamped) {
    Eigen::Vector3f e34c = v3 + e34cp * v43;
    float e12cp = std::clamp(v21.dot(e34c - v1) / v21dv21, 0.0f, 1.0f);
    Eigen::Vector3f e12c = v1 + e12cp * v21;
    Eigen::Vector3f n = e12c - e34c;
    float dist2 = n.squaredNorm();
    if (dist2 > h * h) {
      return std::nullopt;
    }
    return n / std::sqrt(dist2);
  }

  // both para are inside
  Eigen::Vector3f e12c = v1 + e12cp * v21;
  Eigen::Vector3f e34c = v3 + e34cp * v43;
  Eigen::Vector3f n = e12c - e34c;
  float dist2 = n.squaredNorm();
  if (dist2 > h * h) {
    return std::nullopt;
  }
  return n / std::sqrt(dist2);
}

std::optional<PointTriangleImpact> point_triangle_ccd(
    Eigen::Ref<const Eigen::Vector3f> p0, Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30, Eigen::Ref<const Eigen::Vector3f> p1,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31, float h, float tol, int refine_it,
    float eps) {
  auto poly = CCDPoly::try_make_ccd_poly(p0, v10, v20, v30, p1, v11, v21, v31,
                                         tol, refine_it, eps);
  if (!poly) {
    return std::nullopt;
  }

  auto toi = poly->solve();
  if (!toi) {
    return std::nullopt;
  }

  PointTriangleImpact im;
  im.p = p0 + toi.value() * (p1 - p0);
  im.v1 = v10 + toi.value() * (v11 - v10);
  im.v2 = v20 + toi.value() * (v21 - v20);
  im.v3 = v30 + toi.value() * (v31 - v30);
  im.toi = *toi;

  auto normal = point_triangle_collision(im.p, im.v1, im.v2, im.v3, h, eps);
  if (!normal) {
    return std::nullopt;
  }

  im.normal = normal.value();
  return im;
}

std::optional<EdgeEdgeImpact> edge_edge_ccd(
    Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30,
    Eigen::Ref<const Eigen::Vector3f> v40,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31,
    Eigen::Ref<const Eigen::Vector3f> v41, float h, float tol, int refine_it,
    float eps) {
  auto poly = CCDPoly::try_make_ccd_poly(v10, v20, v30, v40, v11, v21, v31, v41,
                                         tol, refine_it, eps);
  if (!poly) {
    return std::nullopt;
  }

  auto toi = poly->solve();
  if (!toi) {
    return std::nullopt;
  }

  EdgeEdgeImpact im;
  im.v1 = v10 + toi.value() * (v11 - v10);
  im.v2 = v20 + toi.value() * (v21 - v20);
  im.v3 = v30 + toi.value() * (v31 - v30);
  im.v4 = v40 + toi.value() * (v41 - v40);
  im.toi = *toi;

  auto normal = edge_edge_collision(im.v1, im.v2, im.v3, im.v4, h, eps);
  if (!normal) {
    return std::nullopt;
  }

  im.normal = normal.value();
  return im;
}

}  // namespace silk
