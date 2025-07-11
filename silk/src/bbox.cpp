#include "bbox.hpp"

#include <Eigen/Core>

namespace silk {

Bbox Bbox::make_zero_bbox() {
  return {.min = Eigen::Vector3f::Zero(), .max = Eigen::Vector3f::Zero()};
}

Bbox Bbox::make_inf_bbox() {
  constexpr float pos_inf = std::numeric_limits<float>::max();
  constexpr float neg_inf = std::numeric_limits<float>::min();
  Eigen::Vector3f max_vec = {pos_inf, pos_inf, pos_inf};
  Eigen::Vector3f min_vec = {neg_inf, neg_inf, neg_inf};
  return {.min = min_vec, .max = max_vec};
}

float Bbox::proximity(const Bbox& a, const Bbox& b) {
  Eigen::Vector3f center_delta = (a.min + a.max) - (b.min + b.max);
  return center_delta.array().abs().sum();
}

Bbox Bbox::merge(const Bbox& a, const Bbox& b) {
  return {.min = a.min.cwiseMin(b.min), .max = a.max.cwiseMax(b.max)};
}

Bbox Bbox::extend(const Bbox& bbox, float margin) {
  assert((margin > 0));
  return {.min = bbox.min.array() - margin, .max = bbox.max.array() + margin};
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

void Bbox::extend_inplace(float margin) {
  assert((margin > 0));

  min = min.array() - margin;
  max = max.array() + margin;
}

}  // namespace silk
