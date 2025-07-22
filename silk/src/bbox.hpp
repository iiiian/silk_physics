#pragma once

#include <Eigen/Core>

namespace silk {

struct Bbox {
  Eigen::Vector3f min;
  Eigen::Vector3f max;

  static float proximity(const Bbox& a, const Bbox& b);
  static Bbox merge(const Bbox& a, const Bbox& b);
  static Bbox pad(const Bbox& bbox, float padding);
  static bool is_disjoint(const Bbox& a, const Bbox& b);
  static bool is_colliding(const Bbox& a, const Bbox& b);

  Eigen::Vector3f center() const;
  bool is_inside(const Bbox& other) const;
  void merge_inplace(const Bbox& other);
  void pad_inplace(float padding);
};

}  // namespace silk
