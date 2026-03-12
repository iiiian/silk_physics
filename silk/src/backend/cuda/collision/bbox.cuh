#pragma once

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Axis-aligned bounding box.
/// The box is invalid if any element of min is smaller or equal to max;
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
  __both__ static inline Bbox merge(const Bbox& a, const Bbox& b) {
    return {vmin(a.min, b.min), vmax(a.max, b.max)};
  }

  /// @brief Create a new bounding box expanded by padding in all directions.
  /// @param bbox Input bounding box
  /// @param padding Distance to expand in each direction (must be positive)
  /// @return Padded bounding box
  __both__ static inline Bbox pad(const Bbox& bbox, float padding) {
    assert(padding > 0);
    return {axpb(1.0f, bbox.min, -padding), axpb(1.0f, bbox.max, padding)};
  }

  /// @brief Check if two bounding boxes overlap.
  /// @param a First bounding box
  /// @param b Second bounding box
  /// @return True if boxes overlap in all three dimensions
  __both__ static inline bool is_colliding(const Bbox& a, const Bbox& b) {
    Vec3f max_min = vmax(a.min, b.min);
    Vec3f min_max = vmin(a.max, b.max);
    return all_lt(max_min, min_max);
  }

  /// Calculate the geometric center of the bounding box.
  /// @return Center point as (min + max) / 2
  __both__ Vec3f center() const { return axpby(0.5f, min, 0.5f, max); }

  /// Check if this bounding box is empty.
  /// @return True if empty
  __both__ bool is_empty() const { return any_geq(min, max); }

  /// Check if this bounding box completely contains another.
  /// @param other Bounding box to test for containment
  /// @return True if other is entirely within this box (strict containment)
  __both__ bool is_inside(const Bbox& other) const {
    return (all_lt(min, other.min) && all_gt(max, other.max));
  }
};

}  // namespace silk::cuda
