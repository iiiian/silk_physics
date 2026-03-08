#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

__both__ Bbox Bbox::merge(const Bbox& a, const Bbox& b) {
  return {vmin(a.min, b.min), vmax(a.max, b.max)};
}

__both__ Bbox Bbox::pad(const Bbox& bbox, float padding) {
  assert(padding > 0);
  return {axpb(1.0f, bbox.min, -padding), axpb(1.0f, bbox.max, padding)};
}

__both__ bool Bbox::is_disjoint(const Bbox& a, const Bbox& b) {
  Vec3f max_min = vmax(a.min, b.min);
  Vec3f min_max = vmin(a.max, b.max);
  return any_lt(max_min, min_max);
}

__both__ bool Bbox::is_colliding(const Bbox& a, const Bbox& b) {
  Vec3f max_min = vmax(a.min, b.min);
  Vec3f min_max = vmin(a.max, b.max);
  return all_lt(max_min, min_max);
}

__both__ Vec3f Bbox::center() const { return axpby(0.5f, min, 0.5f, max); }

__both__ bool Bbox::is_empty() const { return any_geq(min, max); }

__both__ bool Bbox::is_inside(const Bbox& other) const {
  return (all_lt(min, other.min) && all_gt(max, other.max));
}

}  // namespace silk::cuda
