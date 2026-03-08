#pragma once

#include <Eigen/Core>
#include <cassert>

namespace silk::cpu {

/// Axis-aligned bounding box representation using min/max corners.
///
/// Provides spatial operations including collision detection, merging, and
/// proximity calculations. All operations assume valid boxes where min <= max
/// in all dimensions.
struct Bbox {
  /// Minimum corner of the bounding box.
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  /// Maximum corner of the bounding box.
  Eigen::Vector3f max = Eigen::Vector3f::Zero();

  /// Calculate distance heuristic between bounding box.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return Bbox proximity heuristic
  static inline float proximity(const Bbox& a, const Bbox& b) {
    // Calculate difference between centers
    Eigen::Vector3f center_delta = (a.min + a.max) - (b.min + b.max);
    return center_delta.array().abs().sum();
  }

  /// Create a new bounding box containing both input boxes.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return Merged bounding box with min(a.min, b.min) and max(a.max, b.max)
  static inline Bbox merge(const Bbox& a, const Bbox& b) {
    return {a.min.cwiseMin(b.min), a.max.cwiseMax(b.max)};
  }

  /// Create a new bounding box expanded by padding in all directions.
  /// @param bbox Input bounding box
  /// @param padding Distance to expand in each direction (must be positive)
  /// @return Padded bounding box
  static inline Bbox pad(const Bbox& bbox, float padding) {
    assert((padding > 0));
    return {bbox.min.array() - padding, bbox.max.array() + padding};
  }

  /// Check if two bounding boxes do not overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes are separated along any axis
  static inline bool is_disjoint(const Bbox& a, const Bbox& b) {
    Eigen::Vector3f max_min = a.min.cwiseMax(b.min);
    Eigen::Vector3f min_max = a.max.cwiseMin(b.max);
    return (max_min.array() > min_max.array()).any();
  }

  /// Check if two bounding boxes overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes overlap in all three dimensions
  static inline bool is_colliding(const Bbox& a, const Bbox& b) {
    return (a.min.x() < b.max.x() && b.min.x() < a.max.x() &&
            a.min.y() < b.max.y() && b.min.y() < a.max.y() &&
            a.min.z() < b.max.z() && b.min.z() < a.max.z());
  }

  /// Calculate the geometric center of the bounding box.
  /// @return Center point as (min + max) / 2
  inline Eigen::Vector3f center() const { return (min + max) / 2; }

  /// Check if this bounding box completely contains another.
  /// @param other Bounding box to test for containment
  /// @return True if other is entirely within this box (strict containment)
  inline bool is_inside(const Bbox& other) const {
    return ((min.array() < other.min.array()).all() &&
            (max.array() > other.max.array()).all());
  }

  /// Expand this bounding box to include another box.
  /// @param other Bounding box to merge into this one
  inline void merge_inplace(const Bbox& other) {
    min = min.cwiseMin(other.min);
    max = max.cwiseMax(other.max);
  }

  /// Expand this bounding box by padding in all directions.
  /// @param padding Distance to expand in each direction (must be positive)
  inline void pad_inplace(float padding) {
    assert((padding > 0));
    min = min.array() - padding;
    max = max.array() + padding;
  }
};

}  // namespace silk::cpu
