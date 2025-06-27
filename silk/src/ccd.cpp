#include "ccd.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "ccd_poly.hpp"
#include "interval.hpp"

namespace silk {

std::optional<float> CCDSolver::coplaner_point_triangle_ccd(
    Eigen::Ref<const Eigen::Vector3f> p0, Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30, Eigen::Ref<const Eigen::Vector3f> p1,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31) const {
  // TODO: handle coplaner point triangle ccd
  return std::nullopt;
  // // in coplaner ccd case, convert the problem to barycentric coordinate
  // auto bary_coor = [](Eigen::Ref<const Eigen::Vector3f> p,
  //                     Eigen::Ref<const Eigen::Vector3f> v1,
  //                     Eigen::Ref<const Eigen::Vector3f> v2,
  //                     Eigen::Ref<const Eigen::Vector3f> v3) ->
  //                     Eigen::Vector2f {
  //   Eigen::Matrix<float, 3, 2> A;
  //   A.col(0) = v1 - v3;
  //   A.col(1) = v2 - v3;
  //   return A.ldlt().solve(p - v3);
  // };
  // Eigen::Vector2f bary_t0 = bary_coor(p0, v10, v20, v30);
  // Eigen::Vector2f bary_t1 = bary_coor(p1, v11, v21, v31);
  // Eigen::Vector2f bary_delta = bary_t1 - bary_t0;
  //
  // float a = bary_delta(0);
  // float b = bary_t0(0);
  // float c = bary_delta(1);
  // float d = bary_t0(1);
  //
  // // valid time interval for 0 < bary1 < 1
  // Interval it1;
  // if (std::abs(a) < eps) {
  //   if (b < 0 || b > 1) {
  //     return std::nullopt;
  //   }
  //   it1 = Interval{0, 1};
  // } else {
  //   it1 = Interval{-b / a, (1 - b) / a};
  // }
  //
  // // valid time interval for 0 < bary2 < 1
  // Interval it2;
  // if (std::abs(c) < eps) {
  //   if (d < 0 || d > 1) {
  //     return std::nullopt;
  //   }
  //   it1 = Interval{0, 1};
  // } else {
  //   it1 = Interval{-d / c, (1 - d) / c};
  // }
  //
  // // valid time interval for bary1 + bary2 < 1
  // Interval it3;
  // float tmp = a + c;
  // if (std::abs(tmp) < eps) {
  //   if (b + d > 1) {
  //     return std::nullopt;
  //   }
  //   it3 = Interval{0, 1};
  // } else {
  //   it3 = Interval{-std::numeric_limits<float>::infinity(), (1 - b - d) /
  //   tmp};
  // }
  //
  // Interval it4 =
  //     it1.intersection(it2).intersection(it3).intersection(Interval{0, 1});
  // if (it4.is_empty()) {
  //   return std::nullopt;
  // }
  // return it4.lower_bound();
}

std::optional<float> CCDSolver::coplaner_edge_edge_ccd(
    Eigen::Ref<const Eigen::Vector3f> v10,
    Eigen::Ref<const Eigen::Vector3f> v20,
    Eigen::Ref<const Eigen::Vector3f> v30,
    Eigen::Ref<const Eigen::Vector3f> v40,
    Eigen::Ref<const Eigen::Vector3f> v11,
    Eigen::Ref<const Eigen::Vector3f> v21,
    Eigen::Ref<const Eigen::Vector3f> v31,
    Eigen::Ref<const Eigen::Vector3f> v41) const {
  // TODO: handle coplaner edge edge ccd
  return std::nullopt;
}

bool point_triangle_collision(Eigen::Ref<const Eigen::Vector3f> p,
                              Eigen::Ref<const Eigen::Vector3f> v1,
                              Eigen::Ref<const Eigen::Vector3f> v2,
                              Eigen::Ref<const Eigen::Vector3f> v3, float h,
                              float eps) {
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
    return false;
  }

  float b1 = (v23dv23 * v13dvp3 - v13dv23 * v23dvp3) / det;
  float b2 = (-v13dv23 * v13dvp3 + v13dv13 * v23dvp3) / det;

  // barycentric coordinate is outside of triangle
  if (b1 < -eps || b1 > 1 + eps || b2 < -eps || b2 > 1 + eps ||
      b1 + b2 > 1 + eps) {
    return false;
  }

  Eigen::Vector3f proj = b1 * v1 + b2 * v2 + (1 - b1 - b2) * v3;
  return ((p - proj).squaredNorm() < h * h);
}

bool edge_edge_collision(Eigen::Ref<const Eigen::Vector3f> v1,
                         Eigen::Ref<const Eigen::Vector3f> v2,
                         Eigen::Ref<const Eigen::Vector3f> v3,
                         Eigen::Ref<const Eigen::Vector3f> v4, float h,
                         float eps) {
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
    return false;
  }

  // parallel edge
  float area2_eps = pow(eps * std::max(v21dv21, v43dv43), 2);
  if (det < area2_eps) {
    float dist2;
    // choose the shorter edge then query the distance from its vertices to
    // another edge
    if (v21dv21 < v43dv43) {
      // v1 against edge v3 v4
      float pa = std::clamp(v43.dot(-v31) / v43dv43, 0.0f, 1.0f);
      Eigen::Vector3f v1c =
          v3 + pa * v43;  // closest point on edge v3 v4 against v1

      // v2 against edge v3 v4
      float pb = std::clamp(v43.dot(v2 - v3) / v43dv43, 0.0f, 1.0f);
      Eigen::Vector3f v2c =
          v3 + pb * v43;  // closest point on edge v3 v4 against v2

      dist2 = std::min((v1 - v1c).squaredNorm(), (v2 - v2c).squaredNorm());

    } else {
      // v3 against edge v1 v2
      float pa = std::clamp(v21.dot(v31) / v21dv21, 0.0f, 1.0f);
      Eigen::Vector3f v3c =
          v1 + pa * v21;  // closest point on edge v1 v2 against v3

      // v4 against edge v1 v2
      float pb = std::clamp(v21.dot(v4 - v1) / v21dv21, 0.0f, 1.0f);
      Eigen::Vector3f v4c =
          v1 + pb * v21;  // closest point on edge v1 v2 against v4

      dist2 = std::min((v3 - v3c).squaredNorm(), (v4 - v4c).squaredNorm());
    }

    return (dist2 < h * h);
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
  auto [is_e12p_clamped, e12p] =
      try_clamp((v43dv43 * v21dv31 - v21dv43 * v43dv31) / det);
  auto [is_e34p_clamped, e34p] =
      try_clamp((v21dv43 * v21dv31 - v21dv21 * v43dv31) / det);

  // both parameter of edge v1 v2 and edge v3 v4 are outside
  if (is_e12p_clamped && is_e34p_clamped) {
    // compute 2 possible collision point pairs then choose the closer one
    // candidate pair a
    Eigen::Vector3f e12c_a = v1 + e12p * v21;
    float e34p_a = std::clamp(v43.dot(e12c_a - v3) / v43dv43, 0.0f, 1.0f);
    Eigen::Vector3f e34c_a = v3 + e34p_a * v43;

    // candidate pair b
    Eigen::Vector3f e34c_b = v3 + e34p * v43;
    float e12p_b = std::clamp(v21.dot(e34c_b - v1) / v21dv21, 0.0f, 1.0f);
    Eigen::Vector3f e12c_b = v1 + e12p_b * v21;

    float dist2a = (e12c_a - e34c_a).squaredNorm();
    float dist2b = (e12c_b - e34c_b).squaredNorm();
    float dist2 = std::min(dist2a, dist2b);
    return (dist2 < h * h);
  }

  // parameter of edge v1 v2 is outside
  if (is_e12p_clamped) {
    Eigen::Vector3f e12c = v1 + e12p * v21;
    float e34p = std::clamp(v43.dot(e12c - v3) / v43dv43, 0.0f, 1.0f);
    Eigen::Vector3f e34c = v3 + e34p * v43;
    float dist2 = (e12c - e34c).squaredNorm();
    return (dist2 < h * h);
  }

  // parameter of edge v3 v4 is outside
  if (is_e34p_clamped) {
    Eigen::Vector3f e34c = v3 + e34p * v43;
    float e12p = std::clamp(v21.dot(e34c - v1) / v21dv21, 0.0f, 1.0f);
    Eigen::Vector3f e12c = v1 + e12p * v21;
    float dist2 = (e12c - e34c).squaredNorm();
    return (dist2 < h * h);
  }

  // both para are inside
  Eigen::Vector3f e12c = v1 + e12p * v21;
  Eigen::Vector3f e34c = v3 + e34p * v43;
  return ((e12c - e34c).squaredNorm() < h * h);
}

bool CCDSolver::point_triangle_ccd(Eigen::Ref<const Eigen::Vector3f> p0,
                                   Eigen::Ref<const Eigen::Vector3f> v10,
                                   Eigen::Ref<const Eigen::Vector3f> v20,
                                   Eigen::Ref<const Eigen::Vector3f> v30,
                                   Eigen::Ref<const Eigen::Vector3f> p1,
                                   Eigen::Ref<const Eigen::Vector3f> v11,
                                   Eigen::Ref<const Eigen::Vector3f> v21,
                                   Eigen::Ref<const Eigen::Vector3f> v31,
                                   float t0, float t1) {
  auto poly = CCDPoly::try_make_ccd_poly(p0, v10, v20, v30, p1, v11, v21, v31,
                                         tol, max_iter, eps);
  if (!poly) {
    return coplaner_point_triangle_ccd(p0, v10, v20, v30, p1, v11, v21, v31)
        .has_value();
  }

  auto toi = poly->solve();
  if (!toi) {
    return false;
  }

  Eigen::Vector3f v1c = v10 + toi.value() * (v11 - v10);
  Eigen::Vector3f v2c = v20 + toi.value() * (v21 - v20);
  Eigen::Vector3f v3c = v30 + toi.value() * (v31 - v30);
  Eigen::Vector3f pc = p0 + toi.value() * (p1 - p0);
  return point_triangle_collision(pc, v1c, v2c, v3c, h, eps);
}

bool CCDSolver::edge_edge_ccd(Eigen::Ref<const Eigen::Vector3f> v10,
                              Eigen::Ref<const Eigen::Vector3f> v20,
                              Eigen::Ref<const Eigen::Vector3f> v30,
                              Eigen::Ref<const Eigen::Vector3f> v40,
                              Eigen::Ref<const Eigen::Vector3f> v11,
                              Eigen::Ref<const Eigen::Vector3f> v21,
                              Eigen::Ref<const Eigen::Vector3f> v31,
                              Eigen::Ref<const Eigen::Vector3f> v41, float t0,
                              float t1) {
  auto poly = CCDPoly::try_make_ccd_poly(v10, v20, v30, v40, v11, v21, v31, v41,
                                         tol, max_iter, eps);
  if (!poly) {
    return coplaner_edge_edge_ccd(v10, v20, v30, v40, v11, v21, v31, v41)
        .has_value();
  }

  auto toi = poly->solve();
  if (!toi) {
    return false;
  }

  Eigen::Vector3f v1c = v10 + toi.value() * (v11 - v10);
  Eigen::Vector3f v2c = v20 + toi.value() * (v21 - v20);
  Eigen::Vector3f v3c = v30 + toi.value() * (v31 - v30);
  Eigen::Vector3f v4c = v40 + toi.value() * (v41 - v40);
  return edge_edge_collision(v1c, v2c, v3c, v4c, h, eps);
}

}  // namespace silk
