#pragma once

#include <Eigen/Core>

namespace silk {

struct Bbox {
 public:
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();

  static float proximity(const Bbox& a, const Bbox& b);
  static Bbox merge(const Bbox& a, const Bbox& b);

  Eigen::Vector3f center() const;
  bool contain(const Bbox& other) const;
  void merge_inplace(const Bbox& other);
};

}  // namespace silk
