#pragma once

#include <cassert>
#include <cstdint>
#include <cub/cub.cuh>
#include <cuda/buffer>
#include <cuda/std/bit>
#include <cuda/std/cmath>
#include <cuda/std/span>
#include <cuda/std/utility>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

// Vector of colliding pairs
template <typename C>
using CollisionCache = cu::device_buffer<ctd::pair<C*, C*>>;

// TODO: no std::function in device code. use function type template para +
// concept
/// Return false to skip testing the pair
// template <typename C>
// using CollisionFilter = std::function<bool(const C&, const C&)>;

// Courtesy to
// https://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
__device__ inline uint64_t magic_split(uint32_t x) {
  constexpr uint32_t LOW_21_BITS_MASK = 0x1fffffU;
  constexpr uint64_t MAGIC_MASK_1 = 0x001f00000000ffffULL;
  constexpr uint64_t MAGIC_MASK_2 = 0x001f0000ff0000ffULL;
  constexpr uint64_t MAGIC_MASK_3 = 0x100f00f00f00f00fULL;
  constexpr uint64_t MAGIC_MASK_4 = 0x10c30c30c30c30c3ULL;
  constexpr uint64_t MAGIC_MASK_5 = 0x1249249249249249ULL;

  uint64_t y = x & LOW_21_BITS_MASK;
  y = (y | y << 32) & MAGIC_MASK_1;
  y = (y | y << 16) & MAGIC_MASK_2;
  y = (y | y << 8) & MAGIC_MASK_3;
  y = (y | y << 4) & MAGIC_MASK_4;
  y = (y | y << 2) & MAGIC_MASK_5;

  return y;
}

__device__ inline uint64_t morton_code_magic_bits(Vec3f origin,
                                                  Vec3f inv_cell_length,
                                                  Bbox bbox) {
  Vec3f pos = bbox.center();

  Vec3u cell;
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    float float_cell = (pos(i) - origin(i)) / inv_cell_length(i);
    cell(i) = static_cast<uint32_t>(ctd::floor(float_cell));
  }

  return magic_split(cell(0)) | magic_split(cell(1)) << 1 |
         magic_split(cell(2)) << 2;
}

__device__ inline int find_split(ctd::span<uint64_t> keys) {
  assert(!keys.empty());

  uint64_t start_key = keys.front();
  uint64_t end_key = keys.back();

  if (start_key == end_key) {
    return -1;
  }

  int zero_num = __clzll(start_key ^ end_key);
  constexpr uint64_t ONES = 0x1111111111111111ULL;
  uint64_t search_key = end_key & (ONES << (63 - zero_num));

  // Binary search.
  int start = 0;
  int end = keys.size();
  while (end > start) {
    int mid = (start + end) / 2;
    if (keys[mid] < search_key) {
      start = mid + 1;
    } else {
      end = mid;
    }
  }
  return start;
}

template <typename T>
class BVHTree {
 public:
  // @brief build the BVH tree. Assumes all bboxes are not empty.
  BVHTree(int num, Bbox root_bbox, cu::device_buffer<T> colliders,
          cu::device_buffer<Bbox> bboxes, CudaRuntime rt)
      : colliders_(rt.stream, rt.mem_resource, num, cu::no_init),
        bboxes_(bboxes),
        sorted_id_(rt.stream, rt.mem_resource, num, cu::no_init) {
    assert(num > 0 && num == colliders.size() && num == bboxes.size());
    assert(!root_bbox.is_empty());

    Vec3f origin = root_bbox.min;
    Vec3f extend = root_bbox.max = root_bbox.min;
    Vec3f inv_cell_length;
    for (int i = 0; i < 3; ++i) {
      constexpr float pow_21 = static_cast<float>(1U << 20);
      float cell_length = extend(i) / pow_21;
      inv_cell_length(i) = (cell_length < 1e-20f) ? 1e20f : 1.0f / cell_length;
    }

    auto morton_codes =
        cu::make_buffer<uint64_t>(rt.stream, rt.mem_resource, num, cu::no_init);
    ctd::span<uint64_t> morton_span = morton_codes;
    ctd::span<Bbox> bbox_span = bboxes_;
    auto compute_morton = [origin, inv_cell_length, morton_span,
                           bbox_span] __device__(int i) {
      morton_span[i] =
          morton_code_magic_bits(origin, inv_cell_length, bbox_span[i]);
    };
    cub::DeviceFor::Bulk(num, compute_morton, rt.stream.get());

    auto id_in =
        cu::make_buffer<int>(rt.stream, rt.mem_resource, num, cu::no_init);
    ctd::span<int> id_in_span = id_in;
    auto fill_id = [id_in_span] __device__(int i) { id_in_span[i] = i; };
    cub::DeviceFor::Bulk(num, fill_id, rt.stream.get());

    auto key_out =
        cu::make_buffer<uint64_t>(rt.stream, rt.mem_resource, num, cu::no_init);

    size_t radix_sort_temp_size;
    cub::DeviceRadixSort::SortPairs(nullptr, radix_sort_temp_size,
                                    morton_codes.data(), key_out.data(),
                                    colliders.data(), colliders_.data(), num, 0,
                                    sizeof(uint64_t) * 8, rt.stream.get());

    auto radix_sort_temp = cu::make_buffer<char>(
        rt.stream, rt.mem_resource, radix_sort_temp_size, cu::no_init);
    cub::DeviceRadixSort::SortPairs(
        radix_sort_temp.data(), radix_sort_temp_size, morton_codes.data(),
        key_out.data(), colliders.data(), colliders_.data(), num, 0,
        sizeof(uint64_t) * 8, rt.stream.get());
  }

 private:
  cu::device_buffer<T> colliders_;
  cu::device_buffer<Bbox> bboxes_;
  cu::device_buffer<int> sorted_id_;
};

}  // namespace silk::cuda
