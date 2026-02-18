#pragma once

#include <cassert>
#include <cstdint>
#include <cuda/buffer>
#include <cuda/std/bit>
#include <cuda/std/cmath>
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
__device__ constexpr uint64_t magic_split(uint32_t x) {
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

__device__ constexpr uint64_t morton_code_magic_bits(Vec3f origin,
                                                     Vec3f inv_cell_length,
                                                     Vec3f pos) {
  Vec3u cell;
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    float float_cell = (pos(i) - origin(i)) / inv_cell_length(i);
    cell(i) = static_cast<uint32_t>(ctd::floor(float_cell));
  }

  return magic_split(cell(0)) | magic_split(cell(1)) << 1 |
         magic_split(cell(2)) << 2;
}

}  // namespace silk::cuda
