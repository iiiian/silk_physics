#include "exact_collision.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace eg = Eigen;

void Triangle::update_gemotry() {
  e1 = v1 - v0;
  e2 = v2 - v1;
  e3 = v0 - v2;

  eg::Vector3f cross = -e1.cross(e3);
  dA = cross.norm();
  n = cross / dA;
}

// barycentric coordinate 0
inline float bary0(const eg::Vector3f& p, const Triangle& t) {
  return t.e2.cross(p - t.v1).norm() / t.dA;
}

// barycentric coordinate 1
inline float bary1(const eg::Vector3f& p, const Triangle& t) {
  return t.e3.cross(p - t.v2).norm() / t.dA;
}

// based on barycentric coordinate
inline bool is_inside_triangle(float b0, float b1, float b2) {
  return (b0 >= 0 && b0 <= 1 && b1 >= 0 && b1 <= 1 && b2 >= 0 && b2 <= 1);
}

// return true if collision exist
bool resolve_vertex_triangle_collision(eg::Vector3f& p, float w, Triangle& t,
                                       float h) {
  if (w == 0 && t.w0 == 0 && t.w1 == 0 && t.w2 == 0) {
    return false;
  }

  // test distance from p to triangle
  eg::Vector3f delta = p - t.v0;
  float dist = t.n.dot(delta);
  if (std::abs(dist) > h) {
    return false;
  }

  // project p onto triangle and test if inside
  eg::Vector3f proj = p - dist * t.n;
  float b0 = bary0(proj, t);
  float b1 = bary1(proj, t);
  float b2 = 1 - b0 - b1;
  if (!is_inside_triangle(b0, b1, b2)) {
    return false;
  }

  // based on XPBD dynamic

  // gradient of constrain
  // eg::Vector3f dCp = t.n;
  // eg::Vector3f dC0 = -t.n + delta.cross(t.e2) / t.dA;
  // eg::Vector3f dC1 = delta.cross(t.e3) / t.dA;
  // eg::Vector3f dC2 = delta.cross(t.e1) / t.dA;
  Eigen::Vector3f dCp = t.n;
  Eigen::Vector3f dC0 = -b0 * t.n;
  Eigen::Vector3f dC1 = -b1 * t.n;
  Eigen::Vector3f dC2 = -b2 * t.n;

  // lambda
  float C = (dist > 0) ? dist - h : dist + h;
  float lambda = C / (dCp.squaredNorm() * w + dC0.squaredNorm() * t.w0 +
                      dC1.squaredNorm() * t.w1 + dC2.squaredNorm() * t.w2);

  p += w * lambda * dCp;
  t.v0 += t.w0 * lambda * dC0;
  t.v1 += t.w1 * lambda * dC1;
  t.v2 += t.w2 * lambda * dC2;

  return true;
}
