#pragma once

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <functional>
#include <span>
#include <vector>

#include "backend/cpu/collision/bbox.hpp"
#include "backend/cpu/collision/bbox_soa.hpp"
#include "backend/cpu/collision/pdqsort.h"

namespace silk::cpu {

// C stands for collider. A collider should have member bbox of type Bbox.

/// Vector of colliding pairs
template <typename C>
using CollisionCache = std::vector<std::pair<C*, C*>>;

/// Collision filter callback. Return true to skip testing.
template <typename C>
using CollisionFilter = std::function<bool(const C&, const C&)>;

/// Compute mean and variance of collider centers for a proxy subset.
/// Returns pair(mean, variance) across x/y/z.
template <typename C>
std::pair<Eigen::Vector3f, Eigen::Vector3f> proxy_mean_variance(
    std::span<const C> colliders, const int* proxies, int proxy_num) {
  assert((proxy_num > 0));

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Vector3f variance = Eigen::Vector3f::Zero();
  for (int i = 0; i < proxy_num; ++i) {
    int p = proxies[i];
    Eigen::Vector3f center = colliders[p].bbox.center();
    mean += center;
    variance += center.cwiseAbs2();
  }
  mean /= proxy_num;
  variance = variance / proxy_num - mean.cwiseAbs2();

  return std::make_pair(std::move(mean), std::move(variance));
}

/// Find SAP main axis for one collider group.
template <typename C>
int sap_optimal_axis(std::span<const C> colliders, const int* proxies,
                     int proxy_num) {
  assert((proxy_num > 0));

  auto [mean, var] = proxy_mean_variance(colliders, proxies, proxy_num);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

/// Find SAP main axis for two collider group.
template <typename C>
int sap_optimal_axis(std::span<const C> colliders_a, const int* proxies_a,
                     int proxy_num_a, std::span<const C> colliders_b,
                     const int* proxies_b, int proxy_num_b) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  auto [mean_a, var_a] =
      proxy_mean_variance(colliders_a, proxies_a, proxy_num_a);
  auto [mean_b, var_b] =
      proxy_mean_variance(colliders_b, proxies_b, proxy_num_b);
  Eigen::Vector3f mean = (proxy_num_a * mean_a + proxy_num_b * mean_b) /
                         (proxy_num_a + proxy_num_b);
  Eigen::Vector3f tmp_a = (mean_a - mean).array().square();
  Eigen::Vector3f tmp_b = (mean_b - mean).array().square();
  Eigen::Vector3f var =
      proxy_num_a * (var_a + tmp_a) + proxy_num_b * (var_b + tmp_b);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

template <typename C>
void sap_sort_proxies(std::span<const C> colliders, int* proxies, int proxy_num,
                      int axis) {
  assert((proxy_num != 0));

  auto comp = [axis, &colliders](int a, int b) -> bool {
    return (colliders[a].bbox.min(axis) < colliders[b].bbox.min(axis));
  };
  pdqsort_branchless(proxies, proxies + proxy_num, comp);
}

/// SAP test collider a against sorted collider group b.
/// If flip is true, return pair [b, a] instead of [a, b].
template <typename C, bool flip = false, typename FilterT>
void sap_sorted_collision(C& ca, std::span<C> colliders_b, const int* proxies_b,
                          int proxy_num_b, int axis, const FilterT& filter,
                          CollisionCache<C>& cache) {
  assert((proxy_num_b != 0));

  int aux_axis_a = (axis + 1) % 3;
  int aux_axis_b = (axis + 2) % 3;
  for (int i = 0; i < proxy_num_b; ++i) {
    int p2 = proxies_b[i];
    C& cb = colliders_b[p2];

    // Sorted order guarantees ca.min(axis) <= cb.min(axis).
    if (ca.bbox.max(axis) < cb.bbox.min(axis)) {
      break;
    }

    if (!(ca.bbox.min(axis) <= cb.bbox.max(axis) &&
          ca.bbox.min(aux_axis_a) <= cb.bbox.max(aux_axis_a) &&
          cb.bbox.min(aux_axis_a) <= ca.bbox.max(aux_axis_a) &&
          ca.bbox.min(aux_axis_b) <= cb.bbox.max(aux_axis_b) &&
          cb.bbox.min(aux_axis_b) <= ca.bbox.max(aux_axis_b))) {
      continue;
    }

    if constexpr (flip) {
      if (filter(cb, ca)) {
        cache.emplace_back(&cb, &ca);
      }
    } else {
      if (filter(ca, cb)) {
        cache.emplace_back(&ca, &cb);
      }
    }
  }
}

template <typename C, typename FilterT>
void sap_sorted_group_self_collision(std::span<C> colliders, const int* proxies,
                                     int proxy_num, int axis,
                                     const FilterT& filter,
                                     CollisionCache<C>& cache) {
  assert((proxy_num > 0));

  for (int i = 0; i < proxy_num - 1; ++i) {
    int p = proxies[i];
    sap_sorted_collision(colliders[p], colliders, proxies + i + 1,
                         proxy_num - i - 1, axis, filter, cache);
  }
}

template <typename C, typename FilterT>
void sap_sorted_group_group_collision(std::span<C> colliders_a,
                                      const int* proxies_a, int proxy_num_a,
                                      std::span<C> colliders_b,
                                      const int* proxies_b, int proxy_num_b,
                                      int axis, const FilterT& filter,
                                      CollisionCache<C>& cache) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    int pa = proxies_a[a];
    int pb = proxies_b[b];

    if (colliders_a[pa].bbox.min(axis) < colliders_b[pb].bbox.min(axis)) {
      sap_sorted_collision<C, false>(colliders_a[pa], colliders_b,
                                     proxies_b + b, proxy_num_b - b, axis,
                                     filter, cache);
      ++a;
    } else {
      // Flip the output pairs to ensure consistant pair order.
      sap_sorted_collision<C, true>(colliders_b[pb], colliders_a, proxies_a + a,
                                    proxy_num_a - a, axis, filter, cache);
      ++b;
    }
  }
}

/// Test query collider (a) against cadidate collider group b.
template <typename C, bool flip = false, typename FilterT>
void sap_sorted_bbox_collision(const BboxSOAView& bboxes_a, int bbox_id_a,
                               C& collider_a, const BboxSOAView& bboxes_b,
                               std::span<C> colliders_b,
                               std::span<const int> proxies_b, int axis,
                               const FilterT& filter,
                               std::vector<int>& overlap_proxies,
                               CollisionCache<C>& cache) {
  Bbox query_bbox{{bboxes_a.min[0][bbox_id_a], bboxes_a.min[1][bbox_id_a],
                   bboxes_a.min[2][bbox_id_a]},
                  {bboxes_a.max[0][bbox_id_a], bboxes_a.max[1][bbox_id_a],
                   bboxes_a.max[2][bbox_id_a]}};
  int overlap_num =
      sap_bbox_overlaps(query_bbox, bboxes_b, proxies_b, axis, overlap_proxies);
  for (int i = 0; i < overlap_num; ++i) {
    C& cb = colliders_b[overlap_proxies[i]];
    if constexpr (flip) {
      if (filter(cb, collider_a)) {
        cache.emplace_back(&cb, &collider_a);
      }
    } else {
      if (filter(collider_a, cb)) {
        cache.emplace_back(&collider_a, &cb);
      }
    }
  }
}

template <typename C, typename FilterT>
void sap_sorted_bbox_group_self_collision(std::span<C> colliders,
                                          std::span<const int> proxies,
                                          const BboxSOAView& bboxes, int axis,
                                          const FilterT& filter,
                                          std::vector<int>& overlap_proxies,
                                          CollisionCache<C>& cache) {
  assert((proxies.size() > 0));

  int proxy_num = proxies.size();
  for (int i = 0; i < proxy_num - 1; ++i) {
    int p = proxies[i];
    sap_sorted_bbox_collision<C, false>(
        bboxes, i, colliders[p], bboxes.subspan(i + 1), colliders,
        proxies.subspan(i + 1), axis, filter, overlap_proxies, cache);
  }
}

template <typename C, typename FilterT>
void sap_sorted_bbox_group_group_collision(
    std::span<C> colliders_a, std::span<const int> proxies_a,
    const BboxSOAView& bboxes_a, std::span<C> colliders_b,
    std::span<const int> proxies_b, const BboxSOAView& bboxes_b, int axis,
    const FilterT& filter, std::vector<int>& overlap_proxies,
    CollisionCache<C>& cache) {
  assert((proxies_a.size() > 0));
  assert((proxies_b.size() > 0));

  int proxy_num_a = proxies_a.size();
  int proxy_num_b = proxies_b.size();
  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    int pa = proxies_a[a];
    int pb = proxies_b[b];

    if (bboxes_a.min[axis][a] < bboxes_b.min[axis][b]) {
      sap_sorted_bbox_collision<C, false>(
          bboxes_a, a, colliders_a[pa], bboxes_b.subspan(b), colliders_b,
          proxies_b.subspan(b), axis, filter, overlap_proxies, cache);
      ++a;
    } else {
      // Flip the output pairs to ensure consistent pair order.
      sap_sorted_bbox_collision<C, true>(
          bboxes_b, b, colliders_b[pb], bboxes_a.subspan(a), colliders_a,
          proxies_a.subspan(a), axis, filter, overlap_proxies, cache);
      ++b;
    }
  }
}

struct KDNode {
  // Tree topology pointers; tree is kept strictly binary.
  KDNode* parent = nullptr;
  KDNode* left = nullptr;
  KDNode* right = nullptr;

  // Spatial bounds. `bbox` encloses the node region; `plane_bbox` encloses
  // only the proxies attached to this node (useful when the node isn’t a leaf).
  Bbox bbox = {};
  Bbox plane_bbox = {};
  int axis = 0;           // split plane axis
  float position = 0.0f;  // split plane position on `axis`
  int proxy_start = 0;    // [start,end) into the global proxies_ array
  int proxy_end = 0;      // [start,end) into the global proxies_ array
  int population = 0;     // subtree proxy count (inclusive of descendants)
  int delay_offset = 0;   // lazily applied delta to start/end while traversing
  int ext_start = 0;      // external collider buffer start (buffer_ indices)
  int ext_end = 0;        // external collider buffer end (buffer_ indices)

  int proxy_num() const {
    assert((proxy_end >= proxy_start));
    return proxy_end - proxy_start;
  }

  int ext_num() const {
    assert((ext_end >= ext_start));
    return ext_end - ext_start;
  }

  bool is_leaf() const {
    // bvh tree is always balanced, testing one child is enough
    return !left;
  }

  bool is_left() const { return (parent && this == parent->left); }
};

template <typename C>
class KDTree {
 private:
  // -------------------------------------------
  // Helper struct for BVTT tree tree traversal
  // -------------------------------------------

  /// Per thread cache during collision detection.
  struct ThreadData {
    CollisionCache<C> cache;
    std::vector<int> overlap_proxies;  // proxy id for overlapping bbox.
  };
  using AllThreadData = tbb::enumerable_thread_specific<ThreadData>;

  struct BVTTComponent {
    KDNode* node = nullptr;
    // True if this component does not include subtree.
    // For internal node, this means we test on plane bbox only. For leaf, this
    // makes no difference.
    bool proxies_only = false;
  };

  struct BVTTPair {
    BVTTComponent a;
    BVTTComponent b;
  };

  struct BVTTWork {
    KDNode* a = nullptr;
    KDNode* b = nullptr;
  };

  // -------------------------------------------

  static constexpr int NODE_PROXY_NUM_THRESHOLD = 1024;

  int collider_num_ = 0;
  std::vector<C> colliders_;

  KDNode* root_ = nullptr;
  std::vector<KDNode*> stack_;  // for tree traversal
  std::vector<int> proxies_;    // in-order layout proxy array
  BboxSOAStore proxy_bboxes_;   // SOA bbox sorted by proxy order.
  AllThreadData thread_data_;   // reusable per-worker query scratch
  std::vector<int> buffer_;     // for both proxies and external colliders

 public:
  KDTree() = default;

  KDTree(const KDTree&) = delete;

  KDTree(KDTree&& other) noexcept { swap(other); }

  ~KDTree() { delete_subtree(root_); }

  KDTree& operator=(const KDTree&) = delete;

  KDTree& operator=(KDTree&& other) noexcept {
    swap(other);
    return *this;
  }

  void init(std::vector<C> colliders) {
    assert(!colliders.empty());

    delete_subtree(root_);
    collider_num_ = colliders.size();
    colliders_ = std::move(colliders);
    proxies_.resize(collider_num_);
    proxy_bboxes_.resize(collider_num_);
    for (int i = 0; i < collider_num_; ++i) {
      proxies_[i] = i;
    }
    buffer_.resize(collider_num_);
    root_ = new KDNode{};
    root_->proxy_start = 0;
    root_->proxy_end = collider_num_;
    root_->population = collider_num_;
  }

  /// Mutable access to stored colliders (ownership kept by the tree).
  std::vector<C>& get_colliders() { return colliders_; }

  /// Read-only access to stored colliders.
  const std::vector<C>& get_colliders() const { return colliders_; }

  /// Update KD-tree structure for the current frame.
  void update(const Bbox& root_bbox) {
    assert(root_);

    root_->bbox = root_bbox;
    lift_unfit_up();
    optimize_structure();
  }

  /// Enumerate potentially colliding pairs within this tree.
  template <typename FilterT>
  void test_self_collision(const FilterT& filter, CollisionCache<C>& cache) {
    assert(root_);

    for (auto& data : thread_data_) {
      data.cache.clear();
      data.overlap_proxies.clear();
    }

    // First prepare for SAP, choose optimal global SAP axis.
    // Compute bbox density per axis.
    Eigen::Vector3f avg_extent = Eigen::Vector3f::Zero();
    for (const C& collider : colliders_) {
      avg_extent += collider.bbox.max - collider.bbox.min;
    }
    avg_extent /= collider_num_;
    Eigen::Vector3f scene_extent = root_->bbox.max - root_->bbox.min;
    int axis;
    // Choose the axis with the lowest projected bbox density.
    (avg_extent.array() / scene_extent.array()).minCoeff(&axis);
    sort_proxy_groups(axis);

    std::vector<BVTTPair> bvtt_stack;
    std::vector<BVTTWork> bvtt_works;
    bvtt_stack.push_back({{root_, false}, {root_, false}});

    // Get BVTT component bbox. If is internal code and proxy only, test plane
    // bbox. Else test node bbox.
    auto bbox = [](const BVTTComponent& component) -> const Bbox& {
      return component.proxies_only && !component.node->is_leaf()
                 ? component.node->plane_bbox
                 : component.node->bbox;
    };

    // Traverse the tree and generate work (pair of nodes to test later).
    while (!bvtt_stack.empty()) {
      auto [a, b] = bvtt_stack.back();
      bvtt_stack.pop_back();

      if (!Bbox::is_colliding(bbox(a), bbox(b))) {
        continue;
      }

      if (!a.proxies_only && a.node->is_leaf()) {
        a.proxies_only = true;
      }
      if (!b.proxies_only && b.node->is_leaf()) {
        b.proxies_only = true;
      }

      if (a.proxies_only && b.proxies_only) {
        bvtt_works.push_back({a.node, b.node});
        continue;
      }

      if (a.node == b.node && !a.proxies_only && !b.proxies_only) {
        if (a.node->proxy_num() > 0) {
          bvtt_stack.push_back({{a.node, true}, {b.node, false}});
        }
        if (!a.node->is_leaf()) {
          bvtt_stack.push_back({{a.node->left, false}, {a.node->left, false}});
          bvtt_stack.push_back({{a.node->left, false}, {a.node->right, false}});
          bvtt_stack.push_back(
              {{a.node->right, false}, {a.node->right, false}});
        }
        continue;
      }

      if (a.proxies_only) {
        if (b.node->proxy_num() > 0) {
          bvtt_stack.push_back({a, {b.node, true}});
        }
        if (!b.node->is_leaf()) {
          bvtt_stack.push_back({a, {b.node->left, false}});
          bvtt_stack.push_back({a, {b.node->right, false}});
        }
      } else if (b.proxies_only) {
        if (a.node->proxy_num() > 0) {
          bvtt_stack.push_back({{a.node, true}, b});
        }
        if (!a.node->is_leaf()) {
          bvtt_stack.push_back({{a.node->left, false}, b});
          bvtt_stack.push_back({{a.node->right, false}, b});
        }
      } else if (a.node->population >= b.node->population) {
        if (a.node->proxy_num() > 0) {
          bvtt_stack.push_back({{a.node, true}, b});
        }
        if (!a.node->is_leaf()) {
          bvtt_stack.push_back({{a.node->left, false}, b});
          bvtt_stack.push_back({{a.node->right, false}, b});
        }
      } else {
        if (b.node->proxy_num() > 0) {
          bvtt_stack.push_back({a, {b.node, true}});
        }
        if (!b.node->is_leaf()) {
          bvtt_stack.push_back({a, {b.node->left, false}});
          bvtt_stack.push_back({a, {b.node->right, false}});
        }
      }
    }

    // Test each node pair using SAP.

    // to solve template resolution problem.
    std::span<C> collider_span(colliders_.data(), colliders_.size());
    auto process_range = [&](const tbb::blocked_range<int>& range) {
      auto& local = thread_data_.local();
      for (int i = range.begin(); i != range.end(); ++i) {
        const BVTTWork& work = bvtt_works[i];
        std::span<const int> proxies_a(proxies_.data() + work.a->proxy_start,
                                       work.a->proxy_num());
        BboxSOAView bboxes_a = proxy_bboxes_.view(work.a->proxy_start);
        if (work.a == work.b) {
          sap_sorted_bbox_group_self_collision(
              collider_span, proxies_a, bboxes_a, axis, filter,
              local.overlap_proxies, local.cache);
        } else {
          std::span<const int> proxies_b(proxies_.data() + work.b->proxy_start,
                                         work.b->proxy_num());
          BboxSOAView bboxes_b = proxy_bboxes_.view(work.b->proxy_start);
          sap_sorted_bbox_group_group_collision(
              collider_span, proxies_a, bboxes_a, collider_span, proxies_b,
              bboxes_b, axis, filter, local.overlap_proxies, local.cache);
        }
      }
    };
    tbb::parallel_for(tbb::blocked_range<int>(0, bvtt_works.size()),
                      process_range);

    for (auto& data : thread_data_) {
      cache.insert(cache.end(), data.cache.begin(), data.cache.end());
    }
  }

  template <typename FilterT>
  static void test_tree_collision(KDTree& ta, KDTree& tb, const FilterT& filter,
                                  CollisionCache<C>& cache) {
    assert(ta.root_ && tb.root_);

    // First prepare for SAP, choose optimal global SAP axis.
    // Compute bbox density per axis.
    Eigen::Vector3f avg_extent = Eigen::Vector3f::Zero();
    for (const C& collider : ta.colliders_) {
      avg_extent += collider.bbox.max - collider.bbox.min;
    }
    for (const C& collider : tb.colliders_) {
      avg_extent += collider.bbox.max - collider.bbox.min;
    }
    avg_extent /= ta.collider_num_ + tb.collider_num_;
    Eigen::Vector3f scene_extent = ta.root_->bbox.max - ta.root_->bbox.min;
    int axis;
    // Choose the axis with the lowest projected bbox density.
    (avg_extent.array() / scene_extent.array()).minCoeff(&axis);
    ta.sort_proxy_groups(axis);
    tb.sort_proxy_groups(axis);

    std::vector<BVTTPair> bvtt_stack;
    std::vector<BVTTWork> bvtt_works;
    bvtt_stack.push_back({{ta.root_, false}, {tb.root_, false}});

    // Get BVTT component bbox. If is internal code and proxy only, test plane
    // bbox. Else test node bbox.
    auto bbox = [](const BVTTComponent& component) -> const Bbox& {
      return component.proxies_only && !component.node->is_leaf()
                 ? component.node->plane_bbox
                 : component.node->bbox;
    };

    // Traverse the tree and generate work (pair of nodes to test later).
    while (!bvtt_stack.empty()) {
      auto [a, b] = bvtt_stack.back();
      bvtt_stack.pop_back();

      if (!Bbox::is_colliding(bbox(a), bbox(b))) {
        continue;
      }

      if (a.proxies_only && b.proxies_only) {
        bvtt_works.push_back({a.node, b.node});
        continue;
      }

      // If A contains only direct proxies, expand B. If B contains only direct
      // proxies, expand A. If both can be expanded, expand the component with
      // the larger population because it is more likely to produce disjoint
      // bboxes.
      if (a.proxies_only) {
        if (b.node->proxy_num() > 0) {
          bvtt_stack.push_back({a, {b.node, true}});
        }
        if (!b.node->is_leaf()) {
          bvtt_stack.push_back({a, {b.node->left, false}});
          bvtt_stack.push_back({a, {b.node->right, false}});
        }
      } else if (b.proxies_only || a.node->population >= b.node->population) {
        if (a.node->proxy_num() > 0) {
          bvtt_stack.push_back({{a.node, true}, b});
        }
        if (!a.node->is_leaf()) {
          bvtt_stack.push_back({{a.node->left, false}, b});
          bvtt_stack.push_back({{a.node->right, false}, b});
        }
      } else {
        if (b.node->proxy_num() > 0) {
          bvtt_stack.push_back({a, {b.node, true}});
        }
        if (!b.node->is_leaf()) {
          bvtt_stack.push_back({a, {b.node->left, false}});
          bvtt_stack.push_back({a, {b.node->right, false}});
        }
      }
    }
    // Test each node pair using SAP.
    AllThreadData thread_data;
    auto process_range = [&](const tbb::blocked_range<int>& range) {
      auto& local = thread_data.local();
      for (int w = range.begin(); w != range.end(); ++w) {
        const BVTTWork& work = bvtt_works[w];
        std::span<const int> proxies_a(ta.proxies_.data() + work.a->proxy_start,
                                       work.a->proxy_num());
        std::span<const int> proxies_b(tb.proxies_.data() + work.b->proxy_start,
                                       work.b->proxy_num());
        BboxSOAView bboxes_a = ta.proxy_bboxes_.view(work.a->proxy_start);
        BboxSOAView bboxes_b = tb.proxy_bboxes_.view(work.b->proxy_start);
        sap_sorted_bbox_group_group_collision<C>(
            ta.colliders_, proxies_a, bboxes_a, tb.colliders_, proxies_b,
            bboxes_b, axis, filter, local.overlap_proxies, local.cache);
      }
    };
    tbb::parallel_for(tbb::blocked_range<int>(0, bvtt_works.size()),
                      process_range);

    for (auto& local : thread_data) {
      cache.insert(cache.end(), local.cache.begin(), local.cache.end());
    }
  }

  /// Clear scratch buffers and per-thread caches (structure remains intact).
  void delete_cache() {
    stack_ = {};
    buffer_ = {};
  }

 private:
  void swap(KDTree& other) noexcept {
    if (this == &other) {
      return;
    }
    std::swap(collider_num_, other.collider_num_);
    std::swap(colliders_, other.colliders_);
    std::swap(root_, other.root_);
    std::swap(stack_, other.stack_);
    std::swap(proxies_, other.proxies_);
    std::swap(proxy_bboxes_, other.proxy_bboxes_);
    std::swap(thread_data_, other.thread_data_);
    std::swap(buffer_, other.buffer_);
  }

  /// Sort all node's proxies along axis.
  void sort_proxy_groups(int axis) {
    std::vector<KDNode*> nodes;
    nodes.reserve(collider_num_ / NODE_PROXY_NUM_THRESHOLD);
    nodes.push_back(root_);
    for (int i = 0; i < nodes.size(); ++i) {
      KDNode* node = nodes[i];
      if (!node->is_leaf()) {
        nodes.push_back(node->left);
        nodes.push_back(node->right);
      }
    }

    std::span<const C> colliders = colliders_;
    auto sort_node = [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i != range.end(); ++i) {
        KDNode* node = nodes[i];
        if (node->proxy_num() > 1) {
          sap_sort_proxies(colliders, proxies_.data() + node->proxy_start,
                           node->proxy_num(), axis);
        }
        /// Collect proxy to SOA layout.
        for (int j = node->proxy_start; j < node->proxy_end; ++j) {
          proxy_bboxes_.set(j, colliders[proxies_[j]].bbox);
        }
      }
    };
    tbb::parallel_for(tbb::blocked_range<int>(0, nodes.size()), sort_node);
  }

  // Ensure `buffer_` can hold at least `num` integers; avoids repeated
  // reallocations during traversal when building external collider lists.
  void ensure_buffer_size(int num) {
    if (buffer_.size() < num) {
      buffer_.resize(num);
    }
  }

  // Iteratively delete a subtree (avoids recursion depth issues); uses
  // `stack_` as scratch.
  void delete_subtree(KDNode* n) {
    if (!n) {
      return;
    }

    size_t init_stack_size = stack_.size();
    stack_.push_back(n);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (!current->is_leaf()) {
        stack_.push_back(current->right);
        stack_.push_back(current->left);
      }
      delete current;
    }
  }

  // find optimal split plane based on mean and variance of bbox center.
  // Plane axis is argmax variance of centers; position is mean on that axis.
  void find_optimal_plane(KDNode* n) const {
    assert((n->proxy_num() > 0));

    std::span<const C> colliders(colliders_.data(), colliders_.size());
    auto [mean, var] = proxy_mean_variance(
        colliders, proxies_.data() + n->proxy_start, n->proxy_num());
    var.maxCoeff(&n->axis);
    n->position = mean(n->axis);
  }

  // Partition `[proxy_start, proxy_end)` so colliders outside `n->bbox` are
  // moved to the left; returns count moved. Used when pushing unfit proxies up.
  int partition_unfit_proxy_left(const KDNode* n) {
    auto is_outside = [n, &c = colliders_](int p) -> bool {
      return !(n->bbox.is_inside(c[p].bbox));
    };

    int* start = proxies_.data() + n->proxy_start;
    int* end = proxies_.data() + n->proxy_end;
    int* new_start = std::partition(start, end, is_outside);

    return new_start - start;
  }

  // Partition so colliders inside `n->bbox` are moved to the left; returns the
  // count of outside proxies (now placed at the right).
  int partition_unfit_proxy_right(const KDNode* n) {
    auto is_inside = [n, &c = colliders_](int p) -> bool {
      return n->bbox.is_inside(c[p].bbox);
    };

    int* start = proxies_.data() + n->proxy_start;
    int* end = proxies_.data() + n->proxy_end;
    int* new_end = std::partition(start, end, is_inside);

    return end - new_end;
  }

  // Shift a proxy block left by `shift_num`, preserving relative order by
  // staging into `buffer_`. Used when lowering left partition into a child.
  void shift_proxy_left(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(int);
    size_t shift_size = shift_num * sizeof(int);
    int* left = proxies_.data() + proxy_start - shift_num;
    int* right = proxies_.data() + proxy_start;
    ensure_buffer_size(proxy_num);

    // copy right chunk to temp buffer
    std::memcpy(buffer_.data(), right, copy_size);
    // shift left chunk right
    std::memmove(left + proxy_num, left, shift_size);
    // copy right chunk back
    std::memcpy(left, buffer_.data(), copy_size);
  }

  // Shift a proxy block right by `shift_num`, preserving relative order.
  void shift_proxy_right(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(int);
    size_t shift_size = shift_num * sizeof(int);
    int* left = proxies_.data() + proxy_start;
    int* right = proxies_.data() + proxy_start + proxy_num;
    ensure_buffer_size(proxy_num);

    // copy left chunk to temp buffer
    std::memcpy(buffer_.data(), left, copy_size);
    // shift right chunk left
    std::memmove(left, right, shift_size);
    // copy left chunk back
    std::memcpy(left + shift_num, buffer_.data(), copy_size);
  }

  // Bottom-up pass: ensure each node’s proxies fit its `bbox`. Proxies that no
  // longer fit are lifted to the parent by adjusting ranges and, when needed,
  // shifting neighbor blocks once per node rather than per object.
  void lift_unfit_up() {
    stack_.clear();

    // prepare for bottom up traverse
    stack_.push_back(root_);
    for (int i = 0; i < stack_.size(); ++i) {
      KDNode* n = stack_[i];
      if (!n->is_leaf()) {
        stack_.push_back(n->right);
        stack_.push_back(n->left);
      }
    }

    // bottom up refit, push unfit node up. root is excluded
    for (int i = stack_.size() - 1; i > 0; --i) {
      KDNode* n = stack_[i];

      if (n->proxy_num() == 0) {
        continue;
      }

      if (n->is_leaf()) {
        if (n->is_left()) {
          int unfit_num = partition_unfit_proxy_right(n);
          n->proxy_end -= unfit_num;
          assert((n->proxy_end <= collider_num_));
          n->parent->proxy_start -= unfit_num;
          assert((n->parent->proxy_start >= 0));
          n->population = n->proxy_num();
        } else {
          int unfit_num = partition_unfit_proxy_left(n);
          n->proxy_start += unfit_num;
          assert((n->proxy_start >= 0));
          n->parent->proxy_end += unfit_num;
          assert((n->parent->proxy_end <= collider_num_));
          n->population = n->proxy_num();
        }
      } else {
        if (n->is_left()) {
          int unfit_num = partition_unfit_proxy_right(n);
          shift_proxy_right(n->proxy_end - unfit_num, unfit_num,
                            n->right->population);

          n->proxy_end -= unfit_num;
          assert((n->proxy_end <= collider_num_));
          n->parent->proxy_start -= unfit_num;
          assert((n->parent->proxy_start >= 0));
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
          n->right->delay_offset = -unfit_num;
        } else {
          int unfit_num = partition_unfit_proxy_left(n);
          shift_proxy_left(n->proxy_start, unfit_num, n->left->population);

          n->proxy_start += unfit_num;
          assert((n->proxy_start >= 0));
          n->parent->proxy_end += unfit_num;
          assert((n->parent->proxy_end <= collider_num_));
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
          n->left->delay_offset = unfit_num;
        }
      }
    }

    if (!root_->is_leaf()) {
      root_->population = root_->proxy_num() + root_->left->population +
                          root_->right->population;
    }
  }

  // Distribute current node proxies across left/middle/right partitions
  // according to the split plane, updating children ranges/populations and
  // applying delayed offsets so subtree ranges remain consistent.
  void filter(KDNode* n) {
    assert((n->left && n->right));
    assert((n->delay_offset == 0));

    // partition proxies by plane.
    // strictly left  -> move to left partition
    // on the plane   -> move to middle partition
    // strictly right -> move to right partition
    int left_end = n->proxy_start;
    int middle_end = n->proxy_end;
    int i = n->proxy_start;
    while (i != middle_end) {
      int p = proxies_[i];

      // strictly left
      if (colliders_[p].bbox.max(n->axis) < n->position) {
        std::swap(proxies_[left_end], proxies_[i]);
        ++left_end;
        ++i;
      }
      // strictly right
      else if (colliders_[p].bbox.min(n->axis) > n->position) {
        std::swap(proxies_[middle_end - 1], proxies_[i]);
        --middle_end;
      }
      // on the plane
      else {
        ++i;
      }
    }

    // move left partition down one node level
    int left_num = left_end - n->proxy_start;
    if (!n->left->is_leaf()) {
      shift_proxy_left(n->proxy_start, left_num, n->left->right->population);
      n->left->right->delay_offset += left_num;
    }
    n->left->proxy_end += left_num;
    assert((n->left->proxy_end + n->left->delay_offset <= collider_num_));
    n->left->population += left_num;

    // move right partition down one node level
    int right_num = n->proxy_end - middle_end;
    if (!n->right->is_leaf()) {
      shift_proxy_right(middle_end, right_num, n->right->left->population);
      n->right->left->delay_offset -= right_num;
    }
    n->right->proxy_start -= right_num;
    assert((n->right->proxy_start + n->right->delay_offset >= 0));
    n->right->population += right_num;

    // update node
    n->proxy_start += left_num;
    assert((n->proxy_start >= 0));
    n->proxy_end -= right_num;
    assert((n->proxy_end <= collider_num_));
  }

  // Update children `bbox` slabs from parent `bbox` and split plane.
  static void set_children_bbox(KDNode* n) {
    assert((n->left && n->right));

    n->left->bbox = n->bbox;
    n->left->bbox.max(n->axis) = n->position;
    n->right->bbox = n->bbox;
    n->right->bbox.min(n->axis) = n->position;
  }

  // find optimal split plane then split the leaf
  // Split an oversized leaf: choose plane via mean/variance, create children,
  // distribute proxies (filter), and set child/plane bounds.
  void split_leaf(KDNode* n) {
    assert(!(n->left || n->right));
    assert((n->proxy_num() > NODE_PROXY_NUM_THRESHOLD));

    find_optimal_plane(n);

    // create new children
    n->left = new KDNode{};
    n->left->parent = n;
    n->left->proxy_start = n->proxy_start;
    assert((n->left->proxy_start >= 0));
    n->left->proxy_end = n->proxy_start;
    assert((n->left->proxy_end <= collider_num_));

    n->right = new KDNode{};
    n->right->parent = n;
    n->right->proxy_start = n->proxy_end;
    assert((n->right->proxy_start >= 0));
    n->right->proxy_end = n->proxy_end;
    assert((n->right->proxy_end <= collider_num_));

    filter(n);
    set_children_bbox(n);
    set_plane_bbox(n);
  }

  // collapse subtree into leaf
  // Collapse an internal subtree back into a leaf by lifting child
  // populations and deleting the subtrees
  void collapse(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    assert((n->proxy_start >= 0));
    n->proxy_end += n->right->population;
    assert((n->proxy_end <= collider_num_));

    delete_subtree(n->left);
    n->left = nullptr;

    delete_subtree(n->right);
    n->right = nullptr;
  }

  // Heuristic from Serpa & Rodrigues 2019: compare normalized expected test
  // count (Cost) against subtree balance. Favor planes that reduce pair count
  // while keeping reasonable balance; otherwise consider
  // translating/collapsing.
  bool evaluate(const KDNode* n) const {
    assert((n->left && n->right));

    int left_num = 0;
    int middle_num = 0;
    int right_num = 0;

    // partition object proxies by plane.
    // strictly left  -> left partition
    // on the plane   -> middle partition
    // strictly right -> right partition
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      int p = proxies_[i];
      // strictly left
      if (colliders_[p].bbox.max(n->axis) < n->position) {
        ++left_num;
      }
      // strictly right
      else if (colliders_[p].bbox.min(n->axis) > n->position) {
        ++right_num;
      }
      // on the plane
      else {
        ++middle_num;
      }
    }

    // combinatory (x/2)
    auto c2 = [](float x) { return 0.5f * x * (x - 1); };

    float pop = static_cast<float>(n->population);
    float pl = static_cast<float>(left_num + n->left->population);
    float pr = static_cast<float>(right_num + n->right->population);
    float pm = static_cast<float>(middle_num);

    float t_min = 2.0f * c2(0.5f * pop);
    float t_max = c2(pop);
    float t = c2(pm) + c2(pl) + c2(pr) + pm * (pl + pr);
    float cost = (t - t_min) / (t_max - t_min);
    float balance = std::min(pl, pr) / (pm + std::max(pl, pr));
    return (cost <= balance);
  }

  // Prepare for plane translation: temporarily pull all proxies up to `n` by
  // editing ranges only; descendants get zero populations and reset ranges.
  void lift_subtree(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    assert((n->proxy_start >= 0));
    n->proxy_end += n->right->population;
    assert((n->proxy_end <= collider_num_));

    size_t init_stack_size = stack_.size();

    // lift left subtree
    stack_.push_back(n->left);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (!current->is_leaf()) {
        stack_.push_back(current->left);
        stack_.push_back(current->right);
      }

      current->proxy_start = n->proxy_start;
      assert((current->proxy_start >= 0));
      current->proxy_end = n->proxy_start;
      assert((current->proxy_end <= collider_num_));
      current->population = 0;
      current->delay_offset = 0;
    }

    // lift right subtree
    stack_.push_back(n->right);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (!current->is_leaf()) {
        stack_.push_back(current->left);
        stack_.push_back(current->right);
      }

      current->proxy_start = n->proxy_end;
      assert((current->proxy_start >= 0));
      current->proxy_end = n->proxy_end;
      assert((current->proxy_end <= collider_num_));
      current->population = 0;
      current->delay_offset = 0;
    }
  }

  // Move split plane toward the current mean on its axis. Safer than picking a
  // new axis; reuses existing topology and favors coherence.
  void translate(KDNode* n) {
    assert((n->proxy_num() != 0));

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      int p = proxies_[i];
      mean += colliders_[p].bbox.center();
    }

    mean /= float(n->proxy_num());
    n->position = mean(n->axis);
  }

  // Build `plane_bbox` by merging proxies that remain at this node.
  void set_plane_bbox(KDNode* n) {
    if (n->proxy_num() == 0) {
      return;
    }

    int p = proxies_[n->proxy_start];
    n->plane_bbox = colliders_[p].bbox;
    for (int i = n->proxy_start + 1; i < n->proxy_end; ++i) {
      p = proxies_[i];
      n->plane_bbox.merge_inplace(colliders_[p].bbox);
    }
  }

  // Pre-order pass that applies delayed offsets, splits oversized leaves,
  // collapses undersized subtrees, evaluates planes, and if needed translates
  // them. This is the idempotent structure optimization phase per frame.
  void optimize_structure() {
    stack_.clear();

    // pre-order traverse the tree
    stack_.push_back(root_);
    while (!stack_.empty()) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      // propagate and apply delay offset
      if (n->delay_offset != 0) {
        if (!n->is_leaf()) {
          n->left->delay_offset += n->delay_offset;
          n->right->delay_offset += n->delay_offset;
        }
        n->proxy_start += n->delay_offset;
        assert((n->proxy_start >= 0));
        n->proxy_end += n->delay_offset;
        assert((n->proxy_end <= collider_num_));
        n->delay_offset = 0;
      }

      // split a leaf if proxy num is too large
      if (n->is_leaf()) {
        if (n->proxy_num() > NODE_PROXY_NUM_THRESHOLD) {
          split_leaf(n);
          stack_.push_back(n->right);
          stack_.push_back(n->left);
        }
        continue;
      }

      // if population of subtree is too small, collapse into leaf node
      if (n->population < NODE_PROXY_NUM_THRESHOLD) {
        collapse(n);
        stack_.push_back(n);
        continue;
      }

      // first try to reuse the original plane
      if (evaluate(n)) {
        filter(n);
        set_children_bbox(n);
        set_plane_bbox(n);
        stack_.push_back(n->right);
        stack_.push_back(n->left);
        continue;
      }

      // original plane is not optimal, try translating the plane
      // int left_num = n->left->population;
      // int right_num = n->right->population;
      lift_subtree(n);
      translate(n);
      if (evaluate(n)) {
        filter(n);
        set_children_bbox(n);
        set_plane_bbox(n);
        stack_.push_back(n->right);
        stack_.push_back(n->left);
        continue;
      }

      // translated plane is still not optimal, collapse the node
      collapse(n);
      stack_.push_back(n);
      // erase(n, (left_num > right_num));
      // stack_.push_back(n);
    }
  }
};

}  // namespace silk::cpu
