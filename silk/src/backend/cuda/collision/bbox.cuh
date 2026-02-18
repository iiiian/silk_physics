#pragma once

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Axis-aligned bounding box.
class Bbox {
 public:
  /// Minimum corner of the bounding box.
  Vec3f min;
  /// Maximum corner of the bounding box.
  Vec3f max;

  /// @brief Create a new bounding box containing device input boxes.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return Merged bounding box with min(a.min, b.min) and max(a.max, b.max)
  __both__ static Bbox merge(const Bbox& a, const Bbox& b);

  /// @brief Create a new bounding box expanded by padding in all directions.
  /// @param bbox Input bounding box
  /// @param padding Distance to expand in each direction (must be positive)
  /// @return Padded bounding box
  __both__ static Bbox pad(const Bbox& bbox, float padding);

  /// @brief Check if two bounding boxes do not overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes are separated along any axis
  __both__ static bool is_disjoint(const Bbox& a, const Bbox& b);

  /// @brief Check if two bounding boxes overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes overlap in all three dimensions
  __both__ static bool is_colliding(const Bbox& a, const Bbox& b);

  /// Calculate the geometric center of the bounding box.
  /// @return Center point as (min + max) / 2
  __both__ Vec3f center() const;

  /// Check if this bounding box completely contains another.
  /// @param other Bounding box to test for containment
  /// @return True if other is entirely within this box (strict containment)
  __both__ bool is_inside(const Bbox& other) const;
};

}  // namespace silk::cuda
