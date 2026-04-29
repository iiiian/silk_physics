#pragma once

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda::collision {

/// @brief Axis-aligned bounding box.
/// The box is invalid if any element of min is smaller or equal to max;
class Bbox {
 public:
  Vec3f min;
  Vec3f max;

  __both__ static inline Bbox merge(const Bbox& a, const Bbox& b) {
    return {vmin(a.min, b.min), vmax(a.max, b.max)};
  }

  __both__ static inline Bbox pad(const Bbox& bbox, float padding) {
    assert(padding > 0);
    return {axpb(1.0f, bbox.min, -padding), axpb(1.0f, bbox.max, padding)};
  }

  /// @brief Check if two bounding boxes overlap.
  __both__ static inline bool is_colliding(const Bbox& a, const Bbox& b) {
    Vec3f max_min = vmax(a.min, b.min);
    Vec3f min_max = vmin(a.max, b.max);
    return all_lt(max_min, min_max);
  }

  /// Compute (min + max) / 2.
  __both__ Vec3f center() const { return axpby(0.5f, min, 0.5f, max); }

  __both__ bool is_empty() const { return any_geq(min, max); }
};

}  // namespace silk::cuda::collision
