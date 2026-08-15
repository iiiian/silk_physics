#pragma once

#include <cuBQL/bvh.h>

// cuBQL's CUDA builder header requires the BVH declarations first.
#include <cuBQL/builder/cuda.h>
#include <cuBQL/traversal/fixedBoxQuery.h>

#include <cassert>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/std/span>
#include <utility>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

template <typename X, typename Y>
using CubqlCollisionPair = ctd::pair<const X*, const Y*>;

template <typename X>
struct CubqlBVHView {
  cuBQL::bvh3f bvh;
  ctd::span<const X> colliders;
};

namespace detail {

void build_cubql_bvh(cuBQL::bvh3f& bvh, const cuBQL::box3f* boxes, int box_num,
                     cudaStream_t stream);
void free_cubql_bvh(cuBQL::bvh3f& bvh, cudaStream_t stream);

__both__ inline cuBQL::box3f to_cubql_box(const Bbox& bbox) {
  return {cuBQL::vec3f(bbox.min(0), bbox.min(1), bbox.min(2)),
          cuBQL::vec3f(bbox.max(0), bbox.max(1), bbox.max(2))};
}

template <typename X>
__global__ void update_cubql_boxes(ctd::span<const X> colliders,
                                   ctd::span<cuBQL::box3f> boxes) {
  int collider_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (collider_id >= colliders.size()) {
    return;
  }
  boxes[collider_id] = to_cubql_box(colliders[collider_id].bbox);
}

template <typename X, typename Y, typename Filter, bool dedup_self>
__global__ void cubql_batch_traversal(ctd::span<const Y> queries,
                                      ctd::span<const uint32_t> query_ids,
                                      CubqlBVHView<X> tree, Filter filter,
                                      DynSpan<CubqlCollisionPair<X, Y>> out) {
  int sorted_query_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (sorted_query_id >= queries.size()) {
    return;
  }

  int query_id =
      query_ids.empty() ? sorted_query_id : query_ids[sorted_query_id];
  cuBQL::box3f query_box = to_cubql_box(queries[query_id].bbox);
  cuBQL::fixedBoxQuery::forEachLeaf(
      [=] __device__(const uint32_t* primitive_ids, int primitive_num) {
        int target_sorted_id = primitive_ids - tree.bvh.primIDs;
        for (int i = 0; i < primitive_num; ++i, ++target_sorted_id) {
          if constexpr (dedup_self) {
            if (target_sorted_id >= sorted_query_id) {
              continue;
            }
          }

          int target_id = primitive_ids[i];
          const Y& query = queries[query_id];
          const X& target = tree.colliders[target_id];
          // fixedBoxQuery reports every primitive in an overlapping leaf, so
          // apply the exact inclusive primitive-box test before emitting it.
          if (!Bbox::is_colliding(target.bbox, query.bbox) ||
              !filter(target, query)) {
            continue;
          }

          cu::atomic_ref<int> fill{*out.fill};
          int output_id = fill.fetch_add(1);
          if (output_id < out.data.size()) {
            out.data[output_id] = ctd::make_pair(&target, &query);
          }
        }
        return CUBQL_CONTINUE_TRAVERSAL;
      },
      tree.bvh, query_box);
}

}  // namespace detail

template <typename X>
class CubqlBVH {
 public:
  CubqlBVH() = default;

  CubqlBVH(const Bbox& root_bbox, cu::device_buffer<X> colliders,
           CudaRuntime rt)
      : colliders_(std::move(colliders)), stream_(rt.stream.get()) {
    (void)root_bbox;
    assert(!colliders_->empty());
    boxes_ = alloc<cuBQL::box3f>(rt, colliders_->size());
    update_boxes(rt);
    detail::build_cubql_bvh(bvh_, boxes_->data(), boxes_->size(), stream_);
  }

  ~CubqlBVH() { destroy(); }

  CubqlBVH(const CubqlBVH&) = delete;
  CubqlBVH& operator=(const CubqlBVH&) = delete;

  CubqlBVH(CubqlBVH&& other) noexcept
      : bvh_(std::exchange(other.bvh_, {})),
        boxes_(std::move(other.boxes_)),
        colliders_(std::move(other.colliders_)),
        stream_(std::exchange(other.stream_, nullptr)) {}

  CubqlBVH& operator=(CubqlBVH&& other) noexcept {
    if (this != &other) {
      destroy();
      bvh_ = std::exchange(other.bvh_, {});
      boxes_ = std::move(other.boxes_);
      colliders_ = std::move(other.colliders_);
      stream_ = std::exchange(other.stream_, nullptr);
    }
    return *this;
  }

  CubqlBVHView<X> view() const {
    return {.bvh = bvh_, .colliders = *colliders_};
  }

  ctd::span<X> get_colliders() { return *colliders_; }

  void update(const Bbox& root_bbox, CudaRuntime rt) {
    (void)root_bbox;
    update_boxes(rt);
    detail::free_cubql_bvh(bvh_, rt.stream.get());
    detail::build_cubql_bvh(bvh_, boxes_->data(), boxes_->size(),
                            rt.stream.get());
  }

  template <typename Filter>
  void test_self_collision(Filter filter,
                           cu::device_buffer<CubqlCollisionPair<X, X>>& out,
                           int& fill, CudaRuntime rt) {
    if (colliders_->empty()) {
      return;
    }

    auto device_fill = alloc<int>(rt, 1, fill);
    DynSpan<CubqlCollisionPair<X, X>> dynamic_output{.fill = device_fill.data(),
                                                     .data = out};
    int grid_num = div_round_up(colliders_->size(), 128);
    ctd::span<const uint32_t> sorted_ids(bvh_.primIDs, bvh_.numPrims);
    detail::cubql_batch_traversal<X, X, Filter, true>
        <<<grid_num, 128, 0, rt.stream.get()>>>(*colliders_, sorted_ids, view(),
                                                filter, dynamic_output);
    check_cuda(cudaGetLastError());

    int old_fill = fill;
    fill = scalar_load(device_fill.data(), rt);
    if (fill > out.size()) {
      resize_buffer(fill + 1, out, rt);
      dynamic_output.data = out;
      scalar_write(device_fill.data(), old_fill, rt);
      detail::cubql_batch_traversal<X, X, Filter, true>
          <<<grid_num, 128, 0, rt.stream.get()>>>(
              *colliders_, sorted_ids, view(), filter, dynamic_output);
      check_cuda(cudaGetLastError());
    }
  }

  template <typename Y, typename Filter>
  void test_ext_collision(ctd::span<const Y> colliders, Filter filter,
                          cu::device_buffer<CubqlCollisionPair<X, Y>>& out,
                          int& fill, CudaRuntime rt) {
    test_ext_collision_impl<Y>(colliders, {}, filter, out, fill, rt);
  }

  template <typename Y, typename Filter>
  void test_ext_collision(const CubqlBVH<Y>& colliders, Filter filter,
                          cu::device_buffer<CubqlCollisionPair<X, Y>>& out,
                          int& fill, CudaRuntime rt) {
    CubqlBVHView<Y> query_tree = colliders.view();
    ctd::span<const uint32_t> sorted_ids(query_tree.bvh.primIDs,
                                         query_tree.bvh.numPrims);
    test_ext_collision_impl<Y>(query_tree.colliders, sorted_ids, filter, out,
                               fill, rt);
  }

 private:
  template <typename Y, typename Filter>
  void test_ext_collision_impl(ctd::span<const Y> colliders,
                               ctd::span<const uint32_t> query_ids,
                               Filter filter,
                               cu::device_buffer<CubqlCollisionPair<X, Y>>& out,
                               int& fill, CudaRuntime rt) {
    if (colliders_->empty() || colliders.empty()) {
      return;
    }

    auto device_fill = alloc<int>(rt, 1, fill);
    DynSpan<CubqlCollisionPair<X, Y>> dynamic_output{.fill = device_fill.data(),
                                                     .data = out};
    int grid_num = div_round_up(colliders.size(), 128);
    detail::cubql_batch_traversal<X, Y, Filter, false>
        <<<grid_num, 128, 0, rt.stream.get()>>>(colliders, query_ids, view(),
                                                filter, dynamic_output);
    check_cuda(cudaGetLastError());

    int old_fill = fill;
    fill = scalar_load(device_fill.data(), rt);
    if (fill > out.size()) {
      resize_buffer(fill + 1, out, rt);
      dynamic_output.data = out;
      scalar_write(device_fill.data(), old_fill, rt);
      detail::cubql_batch_traversal<X, Y, Filter, false>
          <<<grid_num, 128, 0, rt.stream.get()>>>(colliders, query_ids, view(),
                                                  filter, dynamic_output);
      check_cuda(cudaGetLastError());
    }
  }

  void update_boxes(CudaRuntime rt) {
    int grid_num = div_round_up(colliders_->size(), 128);
    detail::update_cubql_boxes<X>
        <<<grid_num, 128, 0, rt.stream.get()>>>(*colliders_, *boxes_);
    check_cuda(cudaGetLastError());
  }

  void destroy() {
    if (bvh_.nodes != nullptr) {
      detail::free_cubql_bvh(bvh_, stream_);
    }
  }

  cuBQL::bvh3f bvh_{};
  Buf<cuBQL::box3f> boxes_;
  Buf<X> colliders_;
  cudaStream_t stream_ = nullptr;
};

}  // namespace silk::cuda
