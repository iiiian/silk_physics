#include "backend/cpu/collision/dcd.hpp"

#include <algorithm>

namespace silk::cpu {

std::optional<std::pair<float, float>> exact_pt_uv(const Eigen::Vector3f& x0,
                                                   const Eigen::Vector3f& x1,
                                                   const Eigen::Vector3f& x2,
                                                   const Eigen::Vector3f& x3,
                                                   float eps) {
  Eigen::Vector3f x21 = x2 - x1;
  Eigen::Vector3f x31 = x3 - x1;
  Eigen::Vector3f x01 = x0 - x1;

  float x21dx21 = x21.dot(x21);
  float x21dx31 = x21.dot(x31);
  float x31dx31 = x31.dot(x31);
  float x21dx01 = x21.dot(x01);
  float x31dx01 = x31.dot(x01);
  float det = x21dx21 * x31dx31 - x21dx31 * x21dx31;

  // Degenerate triangle; ignore.
  float area_eps = eps * std::max(x21dx21, x31dx31);
  if (det < area_eps * area_eps) {
    return std::nullopt;
  }

  // Barycentric (u, v) of point projection w.r.t. (x2, x3).
  float b1 = (x31dx31 * x21dx01 - x21dx31 * x31dx01) / det;
  float b2 = (x21dx21 * x31dx01 - x21dx31 * x21dx01) / det;
  if (b1 < 0.0f || b2 < 0.0f || b1 + b2 > 1.0f) {
    return std::nullopt;
  }

  return std::make_pair(b1, b2);
}

std::optional<std::pair<float, float>> exact_ee_uv(const Eigen::Vector3f& x0,
                                                   const Eigen::Vector3f& x1,
                                                   const Eigen::Vector3f& x2,
                                                   const Eigen::Vector3f& x3,
                                                   float eps) {
  const Eigen::Vector3f& p1 = x0;
  const Eigen::Vector3f& q1 = x1;
  const Eigen::Vector3f& p2 = x2;
  const Eigen::Vector3f& q2 = x3;

  Eigen::Vector3f d1 = q1 - p1;
  Eigen::Vector3f d2 = q2 - p2;
  Eigen::Vector3f r = p1 - p2;
  float a = d1.dot(d1);
  float b = d1.dot(d2);
  float c = d1.dot(r);
  float e = d2.dot(d2);
  float f = d2.dot(r);

  // If either edge is degenerate, skip.
  if (a < eps || e < eps) {
    return std::nullopt;
  }

  float denominator = a * e - b * b;
  float u = denominator > eps
                ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f)
                : 0.0f;

  float v = (b * u + f) / e;
  if (v < 0.0f) {
    v = 0.0f;
    u = std::clamp(-c / a, 0.0f, 1.0f);
  } else if (v > 1.0f) {
    v = 1.0f;
    u = std::clamp((b - c) / a, 0.0f, 1.0f);
  }
  return std::make_pair(u, v);
}

}  // namespace silk::cpu
