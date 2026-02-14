#pragma once

#include <Eigen/Core>

namespace silk::cpu {

/// Axis-aligned bounding box representation using min/max corners.
///
/// Provides spatial operations including collision detection, merging, and
/// proximity calculations. All operations assume valid boxes where min <= max in
/// all dimensions.
struct Bbox {
  /// Minimum corner of the bounding box.
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  /// Maximum corner of the bounding box.
  Eigen::Vector3f max = Eigen::Vector3f::Zero();

  /// Calculate distance heuristic between bounding box.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return Bbox proximity heuristic
  static float proximity(const Bbox& a, const Bbox& b);

  /// Create a new bounding box containing both input boxes.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return Merged bounding box with min(a.min, b.min) and max(a.max, b.max)
  static Bbox merge(const Bbox& a, const Bbox& b);

  /// Create a new bounding box expanded by padding in all directions.
  /// @param bbox Input bounding box
  /// @param padding Distance to expand in each direction (must be positive)
  /// @return Padded bounding box
  static Bbox pad(const Bbox& bbox, float padding);

  /// Check if two bounding boxes do not overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes are separated along any axis
  static bool is_disjoint(const Bbox& a, const Bbox& b);

  /// Check if two bounding boxes overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes overlap in all three dimensions
  static bool is_colliding(const Bbox& a, const Bbox& b);

  /// Calculate the geometric center of the bounding box.
  /// @return Center point as (min + max) / 2
  Eigen::Vector3f center() const;

  /// Check if this bounding box completely contains another.
  /// @param other Bounding box to test for containment
  /// @return True if other is entirely within this box (strict containment)
  bool is_inside(const Bbox& other) const;

  /// Expand this bounding box to include another box.
  /// @param other Bounding box to merge into this one
  void merge_inplace(const Bbox& other);

  /// Expand this bounding box by padding in all directions.
  /// @param padding Distance to expand in each direction (must be positive)
  void pad_inplace(float padding);
};

}  // namespace silk::cpu
