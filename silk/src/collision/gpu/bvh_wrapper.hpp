#pragma once

#include <cuBQL/builder/cuda.h>
#include <cuBQL/bvh.h>

#include <cassert>
#include <vector>

#include "collision/gpu/cuda_utils.hpp"

namespace silk::gpu {

template <typename C>
class BVHWrapper {
 public:
  int collider_num = 0;
  cuBQL::box3f* d_bboxes = nullptr;
  C* d_colliders = nullptr;
  cuBQL::BinaryBVH<float, 3> d_bvh;

 public:
  BVHWrapper() = default;
  BVHWrapper(const std::vector<cuBQL::box3f>& bboxes,
             const std::vector<C>& colliders) {
    assert(bboxes.size() == colliders.size());

    BVH t;
    t.collider_num = bboxes.size();
    t.d_bboxes = host_vector_to_device(bboxes);
    t.d_colliders = host_vector_to_device(colliders);

    return t;
  }

  BVHWrapper(const BVHWrapper& other) = delete;
  BVHWrapper(BVHWrapper&& other) noexcept { swap(other); }

  BVHWrapper& operator=(const BVHWrapper& other) = delete;
  BVHWrapper& operator=(BVHWrapper&& other) {
    swap(other);
    return *this;
  }

  ~BVHWrapper() {
    if (d_bboxes) {
      cudaFree(d_bboxes);
    }
    if (d_colliders) {
      cudaFree(d_colliders);
    }
    cuBQL::cuda::free(d_bvh);
  }

  void build() { cuBQL::gpuBuilder(d_bvh, bboxes, collider_num); }

 private:
  void swap(BVHWrapper& other) {
    if (this == &other) {
      return;
    }
    std::swap(collider_num, other.collider_num);
    std::swap(d_bboxes, other.d_bboxes);
    std::swap(d_colliders, other.d_colliders);
    std::swap(d_bvh, other.d_bvh);
  }
};

}  // namespace silk::gpu
