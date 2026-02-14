#pragma once

#include <Eigen/Core>

namespace silk::cuda {

/// @brief Closed numeric interval [lower, upper].
///
/// Represents a 1D range with the invariant lower ≤ upper. Used for
/// parameter domains in CCD (time and barycentric coordinates).
class Interval {
 public:
  float lower;
  float upper;

 public:
  Interval() = default;
  Interval(float lower, float upper);

  /// @brief Split the interval at its midpoint.
  /// @return Pair of left and right halves sharing the midpoint.
  std::pair<Interval, Interval> bisect() const;
};

}  // namespace silk::cuda
