#include "bbox.hpp"

#include <Eigen/Core>

namespace silk {

float Bbox::proximity(const Bbox& a, const Bbox& b) {
  Eigen::Vector3f center_delta = (a.min + a.max) - (b.min + b.max);
  return center_delta.array().abs().sum();
}

Bbox Bbox::merge(const Bbox& a, const Bbox& b) {
  return {.min = a.min.cwiseMin(b.min), .max = a.max.cwiseMax(b.max)};
}

Eigen::Vector3f Bbox::center() const { return (min + max) / 2; }

bool Bbox::contain(const Bbox& other) const {
  return ((min.array() < other.min.array()).all() &&
          (max.array() > other.max.array()).all());
}

void Bbox::merge_inplace(const Bbox& other) {
  min = min.cwiseMin(other.min);
  max = max.cwiseMax(other.max);
}

}  // namespace silk
