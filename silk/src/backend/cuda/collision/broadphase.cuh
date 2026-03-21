#pragma once

#include <cassert>
#include <concepts>
#include <cstdint>
#include <cub/cub.cuh>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/std/array>
#include <cuda/std/bit>
#include <cuda/std/cmath>
#include <cuda/std/span>
#include <cuda/std/utility>
#include <cuda/warp>
#include <functional>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Collider type has member bbox of type Bbox.
template <typename T>
concept IsCollider = requires(T t) {
  { t.bbox } -> std::same_as<Bbox>;
};

/// @brief Filter callback takes a pair of collider and return true if ignored.
///
/// This function is called on device.
template <typename T, typename C>
concept IsOnFilterCallback =
    std::convertible_to<std::function<bool(const C&, const C&)>, T>;

template <typename C>
  requires IsCollider<C>
using CandidatePairs = AtomicBuffer<ctd::pair<const C*, const C*>>;

struct BVHNode {
  Bbox bbox;
};

template <typename C>
  requires IsCollider<C>
struct OIBVHTreeView {
  uint32_t max_depth;
  uint32_t skipped_depth;
  uint32_t vleaf_num;
  ctd::span<const C> colliders;
  ctd::span<const int> collider_ids;
  ctd::span<const BVHNode> nodes;
};

template <typename C>
  requires IsCollider<C>
class OIBVHTree {
 private:
  static constexpr uint32_t MAX_SKIP_LEVEL = 1;

  uint32_t max_depth_ = 0;
  uint32_t skipped_depth_ = 0;
  uint32_t vleaf_num_ = 0;
  cu::device_buffer<C> colliders_;
  cu::device_buffer<int> collider_ids_;
  cu::device_buffer<BVHNode> nodes_;

 public:
  OIBVHTree() = default;

  /// @brief Build the BVH tree. Assumes all bboxes are not empty.
  static OIBVHTree make(Bbox root_bbox, cu::device_buffer<C> colliders,
                        CudaRuntime rt);

  /// @brief Get non-owning read only view.
  OIBVHTreeView<C> view() const {
    return {.max_depth = max_depth_,
            .skipped_depth = skipped_depth_,
            .vleaf_num = vleaf_num_,
            .colliders = colliders_,
            .collider_ids = collider_ids_,
            .nodes = nodes_};
  }

  /// @brief Test self collision.
  template <typename OnFilter>
    requires IsOnFilterCallback<OnFilter, C>
  void test_self_collision(const OnFilter& on_filter, CudaRuntime rt,
                           CandidatePairs<C>& out);

  /// @brief Test tree-tree collision.
  template <typename OnFilter>
    requires IsOnFilterCallback<OnFilter, C>
  void test_tree_collision(const OIBVHTree& other, const OnFilter& on_filter,
                           CudaRuntime rt, CandidatePairs<C>& out);
};

namespace detail {

/// @brief Split and interleave 21 bit uint32.
///
/// https://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
__both__ inline uint64_t magic_split(uint32_t x) {
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

/// @brief Compute morton code of a bbox.
__device__ inline uint64_t morton_code_magic_bits(Vec3f origin,
                                                  Vec3f inv_cell_length,
                                                  Bbox bbox) {
  Vec3f pos = bbox.center();
  Vec3u cell;
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    constexpr uint32_t CELL_ID_MAX = (1u << 21u) - 1u;

    float float_cell = (pos(i) - origin(i)) * inv_cell_length(i);
    float_cell = ctd::clamp(float_cell, 0.0f, static_cast<float>(CELL_ID_MAX));
    cell(i) = static_cast<uint32_t>(ctd::floor(float_cell));
  }

  return magic_split(cell(0)) | (magic_split(cell(1)) << 1u) |
         (magic_split(cell(2)) << 2u);
}

/// @brief Compute the number of real node of certain tree level.
__both__ inline uint32_t compute_level_rnode_num(uint32_t depth,
                                                 uint32_t max_depth,
                                                 uint32_t vleaf_num) {
  assert(depth <= max_depth);

  uint32_t lv_node_num = (1u << depth);
  uint32_t lv_vnode_num = vleaf_num >> (max_depth - depth);
  return lv_node_num - lv_vnode_num;
}

/// @brief Compute node index from level node index.
__both__ inline uint32_t compute_node_id(uint32_t lv_node_id, uint32_t depth) {
  return ((1u << depth) | lv_node_id) - 1u;
}

/// @brief Compute node memory offset from node index.
__both__ inline uint32_t compute_mem_offset(uint32_t node_id, uint32_t depth,
                                            uint32_t max_depth,
                                            uint32_t vleaf_num) {
  // virtual leaf num one level shallower than node.
  uint32_t lv_vleaf_num = vleaf_num >> (max_depth - depth);
  lv_vleaf_num = lv_vleaf_num >> 1u;

  uint32_t vnode_num = (lv_vleaf_num << 1u) - ctd::popcount(lv_vleaf_num);
  return node_id - vnode_num;
}

/// @brief Compute bbox for true leaves after accounting for skipped levels.
template <typename C>
__global__ void init_leaf_bbox(uint32_t node_depth, uint32_t max_depth,
                               uint32_t vleaf_num, ctd::span<const C> colliders,
                               ctd::span<const int> collider_ids,
                               ctd::span<BVHNode> nodes) {
  uint32_t tid = blockDim.x * blockIdx.x + threadIdx.x;

  uint32_t lv_rnode_num =
      compute_level_rnode_num(node_depth, max_depth, vleaf_num);

  if (tid < lv_rnode_num) {
    uint32_t node_mem_offset = compute_mem_offset(
        compute_node_id(tid, node_depth), node_depth, max_depth, vleaf_num);
    BVHNode& node = nodes[node_mem_offset];

    // Compute index range.
    uint32_t id_num = collider_ids.size();
    uint32_t id_start = tid * (1u << (max_depth - node_depth));
    uint32_t id_end = (tid + 1u) * (1u << (max_depth - node_depth));
    id_end = min(id_end, id_num);

    assert(id_end > id_start);
    node.bbox = colliders[collider_ids[id_start]].bbox;
    for (int i = id_start + 1; i < id_end; ++i) {
      node.bbox = Bbox::merge(node.bbox, colliders[collider_ids[i]].bbox);
    }
  }
}

/// @brief Propagate and refit node bbox upward.
///
/// Designed for block size of 128. Propagate 8 levels upward.
template <typename C>
  requires IsCollider<C>
__global__ void propagate_bbox_128(uint32_t starting_depth, uint32_t max_depth,
                                   uint32_t vleaf_num,
                                   ctd::span<BVHNode> nodes) {
  assert(blockDim.x == 128);

  uint32_t lid = threadIdx.x % 32;                       // lane id
  uint32_t wid = threadIdx.x / 32;                       // warp id
  uint32_t tid = blockDim.x * blockIdx.x + threadIdx.x;  // thread id

  // 1. 1 level reduction from global mem.
  // 2. 5 levels warp reduction.
  // 3. 2 levels block reduction.
  int max_warp_up = min(starting_depth, 5);
  int max_block_up = min(starting_depth - max_warp_up, 2);
  uint32_t node_depth = starting_depth;
  Bbox bbox;

  // First iteration. Load child bbox from global memory.
  {
    uint32_t lv_rnode_num =
        compute_level_rnode_num(node_depth, max_depth, vleaf_num);
    uint32_t lv_node_id = tid;
    if (lv_node_id < lv_rnode_num) {
      uint32_t left_lv_node_id = lv_node_id << 1u;
      uint32_t child_lv_rnode_num =
          compute_level_rnode_num(node_depth + 1u, max_depth, vleaf_num);

      bool is_left_real = (left_lv_node_id < child_lv_rnode_num);
      bool is_right_real = (left_lv_node_id + 1u < child_lv_rnode_num);
      assert(is_left_real || is_right_real);

      uint32_t left_node_id = compute_node_id(left_lv_node_id, node_depth + 1u);
      uint32_t left_mem_offset = compute_mem_offset(
          left_node_id, node_depth + 1u, max_depth, vleaf_num);

      if (is_left_real && is_right_real) {
        bbox = Bbox::merge(nodes[left_mem_offset].bbox,
                           nodes[left_mem_offset + 1u].bbox);
      } else if (is_left_real) {
        bbox = nodes[left_mem_offset].bbox;
      } else {
        bbox = nodes[left_mem_offset + 1u].bbox;
      }

      uint32_t node_mem_offset =
          compute_mem_offset(compute_node_id(lv_node_id, node_depth),
                             node_depth, max_depth, vleaf_num);
      nodes[node_mem_offset].bbox = bbox;
    }
  }

  --node_depth;

  // Warp level propagation.
  for (int i = 1; i <= max_warp_up; ++i) {
    uint32_t delta = 1u << (i - 1);
    Bbox shuffled_bbox = cu::device::warp_shuffle_down(bbox, delta);

    uint32_t lv_rnode_num =
        compute_level_rnode_num(node_depth, max_depth, vleaf_num);

    // Test if lane id % 2^i == 0.
    if ((lid & ((1u << i) - 1u)) == 0u) {
      uint32_t lv_node_id = tid >> i;

      if (lv_node_id < lv_rnode_num) {
        uint32_t left_lv_node_id = lv_node_id << 1u;
        uint32_t child_lv_rnode_num =
            compute_level_rnode_num(node_depth + 1u, max_depth, vleaf_num);

        bool is_left_real = (left_lv_node_id < child_lv_rnode_num);
        bool is_right_real = ((left_lv_node_id + 1u) < child_lv_rnode_num);
        assert(is_left_real || is_right_real);

        if (is_left_real && is_right_real) {
          bbox = Bbox::merge(bbox, shuffled_bbox);
        } else if (is_right_real) {
          bbox = shuffled_bbox;
        }
        // If only is_left_real, bbox remain unchanged.

        uint32_t node_mem_offset =
            compute_mem_offset(compute_node_id(lv_node_id, node_depth),
                               node_depth, max_depth, vleaf_num);
        nodes[node_mem_offset].bbox = bbox;
      }
    }

    --node_depth;
  }

  // Block level propagation.

  // fill temp storage if block propagation is required.
  __shared__ Bbox bbox_temp[4];
  if (max_block_up > 0) {
    if (lid == 0u) {
      bbox_temp[wid] = bbox;
    }
    __syncthreads();
  }

  for (int i = 0; i < max_block_up; ++i) {
    uint32_t wid_mod = wid & ((1u << (i + 1)) - 1u);
    if (lid == 0u && (wid_mod == 0u)) {
      uint32_t lv_rnode_num =
          compute_level_rnode_num(node_depth, max_depth, vleaf_num);

      uint32_t lv_node_id = tid >> (max_warp_up + i + 1u);

      if (lv_node_id < lv_rnode_num) {
        uint32_t left_lv_node_id = lv_node_id << 1u;
        uint32_t child_lv_rnode_num =
            compute_level_rnode_num(node_depth + 1u, max_depth, vleaf_num);

        bool is_left_real = (left_lv_node_id < child_lv_rnode_num);
        bool is_right_real = ((left_lv_node_id + 1u) < child_lv_rnode_num);
        assert(is_left_real || is_right_real);

        uint32_t left_bbox_id = wid;
        uint32_t right_bbox_id = wid + (1u << i);

        if (is_left_real && is_right_real) {
          bbox_temp[wid] =
              Bbox::merge(bbox_temp[left_bbox_id], bbox_temp[right_bbox_id]);
        } else if (is_left_real) {
          bbox_temp[wid] = bbox_temp[left_bbox_id];
        } else {
          bbox_temp[wid] = bbox_temp[right_bbox_id];
        }

        uint32_t node_mem_offset =
            compute_mem_offset(compute_node_id(lv_node_id, node_depth),
                               node_depth, max_depth, vleaf_num);
        nodes[node_mem_offset].bbox = bbox_temp[wid];
      }
    }

    --node_depth;
    __syncthreads();
  }
}

/// @brief Stack designed for simple and small type.
template <typename T, int N>
class SimpleStack {
 public:
  __both__ void push(T value) {
    assert(fill_ < N);
    data_[fill_] = value;
    ++fill_;
  }

  __both__ T pop() {
    assert(fill_ != 0);
    --fill_;
    return data_[fill_];
  }

  __both__ bool is_empty() { return fill_ == 0; }

 private:
  int fill_ = 0;
  ctd::array<T, N> data_;
};

/// @brief Traverse collider through the tree.
template <typename C, typename OnFilter, bool dedup_self>
__device__ void traverse(const C& collider, int dedup_id,
                         OIBVHTreeView<C> oibvh_tree, const OnFilter& on_filter,
                         CandidatePairs<C>& out) {
  const Bbox& bbox = collider.bbox;
  assert(!bbox.is_empty());

  auto& t = oibvh_tree;

  SimpleStack<uint32_t, 32> stack;
  stack.push(0u);

  while (!stack.is_empty()) {
    uint32_t node_id = stack.pop();
    uint32_t depth = 31u - ctd::countl_zero(node_id + 1u);
    uint32_t lv_node_id = node_id - ((1u << depth) - 1u);
    uint32_t lv_rnode_num =
        compute_level_rnode_num(depth, t.max_depth, t.vleaf_num);

    // Skip virtual node.
    if (lv_node_id >= lv_rnode_num) {
      continue;
    }

    uint32_t node_mem_offset =
        compute_mem_offset(node_id, depth, t.max_depth, t.vleaf_num);
    auto& node_bbox = t.nodes[node_mem_offset].bbox;

    if (!Bbox::is_colliding(bbox, node_bbox)) {
      continue;
    }

    // Reach leaf.
    if (depth == t.skipped_depth) {
      // Compute index range.
      uint32_t leaf_cid_num = oibvh_tree.collider_ids.size();
      uint32_t leaf_cid_start = lv_node_id * (1u << (t.max_depth - depth));
      uint32_t leaf_cid_end = (lv_node_id + 1u) * (1u << (t.max_depth - depth));
      leaf_cid_end = min(leaf_cid_end, leaf_cid_num);

      // Test each leaf colliders.
      assert(leaf_cid_end > leaf_cid_start);
      for (int i = leaf_cid_start; i < leaf_cid_end; ++i) {
        // For self collision test only.
        if constexpr (dedup_self) {
          if (dedup_id <= i) {
            continue;
          }
        }
        const C& leaf_collider = t.colliders[t.collider_ids[i]];
        if (!Bbox::is_colliding(bbox, leaf_collider.bbox)) {
          continue;
        }
        if (!on_filter(collider, leaf_collider)) {
          continue;
        }

        // Skip if out buffer is full.
        int out_idx = out.counter.fetch_add(1);
        if (out_idx < out.data.size()) {
          out.data[out_idx] = ctd::make_pair(&collider, &leaf_collider);
        }
      }
    }
    // Descend.
    else {
      // Left child first.
      uint32_t left = 2u * node_id + 1u;
      uint32_t right = 2u * node_id + 2u;
      stack.push(right);
      stack.push(left);
    }
  }
}

}  // namespace detail

template <typename C>
  requires IsCollider<C>
OIBVHTree<C> OIBVHTree<C>::make(Bbox root_bbox, cu::device_buffer<C> colliders,
                                CudaRuntime rt) {
  assert(colliders.size() != 0);
  assert(!root_bbox.is_empty());
  static_assert(sizeof(BVHNode) == 32, "Unexpected BVHNode size.");

  // Compute cell dimension for morton code.
  Vec3f origin = root_bbox.min;
  Vec3f extend = vsub(root_bbox.max, root_bbox.min);
  Vec3f inv_cell_length;
  for (int i = 0; i < 3; ++i) {
    constexpr float pow_21 = static_cast<float>(1u << 21);
    float cell_length = extend(i) / pow_21;
    inv_cell_length(i) = (cell_length < 1e-20f) ? 1e20f : 1.0f / cell_length;
  }

  // Fill collider id map and compute morton code.
  int collider_num = colliders.size();
  auto unsort_ids = cu::make_buffer<int>(rt.stream, rt.mem_resource,
                                         collider_num, cu::no_init);
  auto unsort_morton = cu::make_buffer<uint64_t>(rt.stream, rt.mem_resource,
                                                 collider_num, cu::no_init);
  auto compute_id_and_morton = [origin, inv_cell_length, id = unsort_ids.data(),
                                c = colliders.data(),
                                m = unsort_morton.data()] __device__(int i) {
    id[i] = i;
    m[i] = morton_code_magic_bits(origin, inv_cell_length, c[i].bbox);
  };
  cub::DeviceFor::Bulk(collider_num, compute_id_and_morton, rt.stream.get());

  // Radix sort leaves by morton code.
  auto morton_out = cu::make_buffer<uint64_t>(rt.stream, rt.mem_resource,
                                              collider_num, cu::no_init);
  auto collider_ids = cu::make_buffer<int>(rt.stream, rt.mem_resource,
                                           collider_num, cu::no_init);

  size_t radix_sort_temp_size;
  cub::DeviceRadixSort::SortPairs(
      nullptr, radix_sort_temp_size, unsort_morton.data(), morton_out.data(),
      unsort_ids.data(), collider_ids.data(), collider_num, 0,
      sizeof(uint64_t) * 8, rt.stream.get());
  auto radix_sort_temp = cu::make_buffer<char>(
      rt.stream, rt.mem_resource, radix_sort_temp_size, cu::no_init);

  cub::DeviceRadixSort::SortPairs(
      radix_sort_temp.data(), radix_sort_temp_size, unsort_morton.data(),
      morton_out.data(), unsort_ids.data(), collider_ids.data(), collider_num,
      0, sizeof(uint64_t) * 8, rt.stream.get());

  // Free temp memory to reduce peak memory usage.
  unsort_ids.destroy();
  unsort_morton.destroy();
  morton_out.destroy();
  radix_sort_temp.destroy();

  // Compute tree statistics. Use unsigned int because we use bitwise
  // operation to compute index.
  uint32_t rleaf_num = static_cast<uint32_t>(collider_num);
  uint32_t leaf_num = ctd::bit_ceil(rleaf_num);
  assert(leaf_num > 0);
  uint32_t max_depth = ctd::countr_zero(leaf_num);
  uint32_t vleaf_num = leaf_num - rleaf_num;

  uint32_t skipped_depth =
      (max_depth > MAX_SKIP_LEVEL) ? max_depth - MAX_SKIP_LEVEL : 0;
  uint32_t skipped_vleaf_num = vleaf_num >> (max_depth - skipped_depth);
  uint32_t skipped_rleaf_num = (1u << skipped_depth) - skipped_vleaf_num;
  uint32_t skipped_rnode_num =
      2u * skipped_rleaf_num - 1u + ctd::popcount(skipped_vleaf_num);

  // Allocate and fill the nodes.
  BVHNode empty_node = {.bbox = {}};
  auto nodes = cu::make_buffer<BVHNode>(rt.stream, rt.mem_resource,
                                        skipped_rnode_num, empty_node);

  assert(skipped_rleaf_num > 0);
  int grid_num = compute_grid_num(skipped_rleaf_num, 128);
  detail::init_leaf_bbox<C><<<grid_num, 128, 0, rt.stream.get()>>>(
      skipped_depth, max_depth, vleaf_num, colliders, collider_ids, nodes);

  for (int i = skipped_depth; i > 0; i -= 8) {
    uint32_t depth = i - 1;
    uint32_t lv_rnode_num =
        detail::compute_level_rnode_num(depth, max_depth, vleaf_num);
    assert(lv_rnode_num > 0);

    int grid_num = compute_grid_num(lv_rnode_num, 128);
    detail::propagate_bbox_128<C><<<grid_num, 128, 0, rt.stream.get()>>>(
        depth, max_depth, vleaf_num, nodes);
  }

  OIBVHTreeView<C> t;
  t.max_depth_ = max_depth;
  t.skipped_depth_ = skipped_depth;
  t.vleaf_num_ = vleaf_num;
  t.colliders_ = std::move(colliders);
  t.collider_ids_ = std::move(collider_ids);
  t.nodes_ = std::move(nodes);

  return t;
}

template <typename C>
  requires IsCollider<C>
template <typename OnFilter>
  requires IsOnFilterCallback<OnFilter, C>
void OIBVHTree<C>::test_self_collision(const OnFilter& on_filter,
                                       CudaRuntime rt, CandidatePairs<C>& out) {
  out.counter = 0;
  if (colliders_.empty()) {
    return;
  }

  // clang-format off
    auto batch_traversal = [&on_filter, &out, tree = view()]
      __device__ (uint32_t i) {
      traverse<OnFilter, true>(
          tree.colliders[tree.collider_ids[i]], i, tree, on_filter, out);
    };
  // clang-format on

  // TODO: launch with block size 128 to improve perf.
  cub::DeviceFor::Bulk(collider_ids_.size(), batch_traversal, rt.stream.get());
  // If buffer overlfow, resize then traverse again.
  if (out.counter > out.data.size()) {
    out.data = cu::make_buffer(rt.stream, rt.mem_resource, out.counter);
    out.counter = 0;
    cub::DeviceFor::Bulk(collider_ids_.size(), batch_traversal,
                         rt.stream.get());
  }
}

template <typename C>
  requires IsCollider<C>
template <typename OnFilter>
  requires IsOnFilterCallback<OnFilter, C>
void OIBVHTree<C>::test_tree_collision(const OIBVHTree& other,
                                       const OnFilter& on_filter,
                                       CudaRuntime rt, CandidatePairs<C>& out) {
  assert(this != &other);

  out.counter = 0;

  if (colliders_.empty() || other.colliders_.empty()) {
    return;
  }

  // clang-format off
    auto batch_traversal = [&on_filter, &out, ta = view(), tb =other.view()]
      __device__ (uint32_t i) {
      traverse<OnFilter, false>(
          ta.colliders[ta.collider_ids[i]], 0, tb, on_filter, out);
    };
  // clang-format on

  cub::DeviceFor::Bulk(collider_ids_.size(), batch_traversal, rt.stream.get());
  // If buffer overlfow, resize then traverse again.
  int candidate_num = out.counter;
  if (candidate_num > out.data.size()) {
    out = AtomicBuffer<C>{candidate_num, rt};
    cub::DeviceFor::Bulk(collider_ids_.size(), batch_traversal,
                         rt.stream.get());
  }
}

}  // namespace silk::cuda
