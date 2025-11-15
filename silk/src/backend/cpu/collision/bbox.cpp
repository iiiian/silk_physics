#include "backend/cpu/collision/bbox.hpp"

#include <Eigen/Core>

namespace silk::cpu {

float Bbox::proximity(const Bbox& a, const Bbox& b) {
  // Calculate difference between centers
  Eigen::Vector3f center_delta = (a.min + a.max) - (b.min + b.max);
  return center_delta.array().abs().sum();
}

Bbox Bbox::merge(const Bbox& a, const Bbox& b) {
  return {a.min.cwiseMin(b.min), a.max.cwiseMax(b.max)};
}

Bbox Bbox::pad(const Bbox& bbox, float padding) {
  assert((padding > 0));
  return {bbox.min.array() - padding, bbox.max.array() + padding};
}

bool Bbox::is_disjoint(const Bbox& a, const Bbox& b) {
  Eigen::Vector3f max_min = a.min.cwiseMax(b.min);
  Eigen::Vector3f min_max = a.max.cwiseMin(b.max);
  return (max_min.array() > min_max.array()).any();
}

bool Bbox::is_colliding(const Bbox& a, const Bbox& b) {
  Eigen::Vector3f max_min = a.min.cwiseMax(b.min);
  Eigen::Vector3f min_max = a.max.cwiseMin(b.max);
  return (max_min.array() < min_max.array()).all();
}

Eigen::Vector3f Bbox::center() const { return (min + max) / 2; }

bool Bbox::is_inside(const Bbox& other) const {
  return ((min.array() < other.min.array()).all() &&
          (max.array() > other.max.array()).all());
}

void Bbox::merge_inplace(const Bbox& other) {
  min = min.cwiseMin(other.min);
  max = max.cwiseMax(other.max);
}

void Bbox::pad_inplace(float padding) {
  assert((padding > 0));
  min = min.array() - padding;
  max = max.array() + padding;
}

}  // namespace silk::cpu
