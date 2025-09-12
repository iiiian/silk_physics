#pragma once

#include <pdqsort.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <functional>
#include <vector>

#include "bbox.hpp"

namespace silk {

/**
 * Broad-phase collision detection using a KD-tree + Sweep-and-Prune (SAP).
 *
 * Overview
 * - Data structure: a variable-depth KD-tree whose internal nodes store a split
 *   plane (axis, position), and whose leaves store ranges of object proxies
 * into a single in-order proxy array. Each object C must expose a member `bbox`
 * of type `Bbox` representing its current AABB.
 * - Update: the tree is updated in-place from the previous frame using a set of
 *   lightweight operators inspired by Serpa & Rodrigues 2019 (KD-tree + SAP):
 *   lift/refit, split, collapse, translate-plane, and a heuristic evaluate step
 *   (Cost vs Balance) to decide whether to keep or rework a node. This favors
 *   temporal coherence and avoids full rebuilds.
 * - Query: candidate pairs are generated per node using SAP along the axis with
 *   the largest variance of collider centers. Node–node and node–external pairs
 *   are handled; work is parallelized with OpenMP tasks and accumulated in per-
 *   thread caches before merging.
 *
 * Key invariants
 * - `proxies_` holds all collider indices in a single array; each KDNode keeps
 *   a half-open range [proxy_start, proxy_end) into this array. A node’s
 *   `population` is the total number of proxies in its subtree.
 * - Internal nodes have valid `axis` and `position` (split plane). Leaves do
 *   not get evaluated for plane quality and are split only when their proxy
 *   count exceeds NODE_PROXY_NUM_THRESHOLD.
 * - `delay_offset` defers edits to child proxy ranges when lifting or moving
 *   proxies so parent edits can be applied lazily during pre-order traversal.
 *
 * Notes
 * - The evaluate() heuristic mirrors the Cost/Balance criteria from Serpa &
 *   Rodrigues, CGF 2019, normalizing the expected number of tests under ideal
 *   and worst splits to decide whether to keep the current plane.
 * - `CollisionFilter` lets callers prune domain-knowledge pairs (e.g. static-
 *   static) before AABB checks without modifying the broad-phase logic.
 * - Callers must update each collider's `bbox` before `update()`.
 */

// C stands for collider. A collider should have member bbox of type Bbox.

template <typename C>
using CollisionCache =
    std::vector<std::pair<C*, C*>>;  // vector of colliding pairs

template <typename C>
using CollisionFilter = std::function<bool(
    const C&, const C&)>;  // return false to skip testing the pair

template <typename C>
/**
 * Compute mean and variance of collider centers for a proxy subset.
 * Returns pair(mean, variance) across x/y/z. Used to pick SAP axis and to
 * translate planes toward the current distribution.
 */
std::pair<Eigen::Vector3f, Eigen::Vector3f> proxy_mean_variance(
    const std::vector<C>& colliders, const int* proxies, int proxy_num) {
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

template <typename C>
/** Select SAP axis as the one with maximal center variance for the group. */
int sap_optimal_axis(const std::vector<C>& colliders, const int* proxies,
                     int proxy_num) {
  assert((proxy_num > 0));

  auto [mean, var] = proxy_mean_variance(colliders, proxies, proxy_num);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

template <typename C>
/** Select SAP axis considering two groups (bipartite), pooling variance. */
int sap_optimal_axis(const std::vector<C>& colliders_a, const int* proxies_a,
                     int proxy_num_a, const std::vector<C>& colliders_b,
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
/** Sort proxies in-place by AABB min on `axis` for SAP. */
void sap_sort_proxies(const std::vector<C>& colliders, int* proxies,
                      int proxy_num, int axis) {
  assert((proxy_num != 0));

  auto comp = [axis, &colliders](int a, int b) -> bool {
    return (colliders[a].bbox.min(axis) < colliders[b].bbox.min(axis));
  };
  pdqsort_branchless(proxies, proxies + proxy_num, comp);
}

template <typename C>
/**
 * Sweep one object against a sorted list on `axis`.
 * Early exits when intervals separate; defers narrow-phase by returning pairs
 * of pointers to colliders; caller may run additional checks.
 */
void sap_sorted_collision(C& ca, std::vector<C>& colliders_b,
                          const int* proxies_b, int proxy_num_b, int axis,
                          CollisionFilter<C> filter, CollisionCache<C>& cache) {
  assert((proxy_num_b != 0));

  for (int i = 0; i < proxy_num_b; ++i) {
    int p2 = proxies_b[i];
    C& cb = colliders_b[p2];

    // axis test
    if (ca.bbox.max(axis) < cb.bbox.min(axis)) {
      break;
    }

    // user provided collision filter
    if (!filter(ca, cb)) {
      continue;
    }

    if (Bbox::is_colliding(ca.bbox, cb.bbox)) {
      cache.emplace_back(&ca, &cb);
    }
  }
}

template <typename C>
/** SAP within one sorted group; each pair passes through `filter`. */
void sap_sorted_group_self_collision(std::vector<C>& colliders,
                                     const int* proxies, int proxy_num,
                                     int axis, CollisionFilter<C> filter,
                                     CollisionCache<C>& cache) {
  assert((proxy_num > 0));

  for (int i = 0; i < proxy_num - 1; ++i) {
    int p = proxies[i];
    sap_sorted_collision(colliders[p], colliders, proxies + i + 1,
                         proxy_num - i - 1, axis, filter, cache);
  }
}

template <typename C>
/** Bipartite SAP between two sorted groups. */
void sap_sorted_group_group_collision(std::vector<C>& colliders_a,
                                      const int* proxies_a, int proxy_num_a,
                                      std::vector<C>& colliders_b,
                                      const int* proxies_b, int proxy_num_b,
                                      int axis, CollisionFilter<C> filter,
                                      CollisionCache<C>& cache) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    int pa = proxies_a[a];
    int pb = proxies_b[b];

    if (colliders_a[pa].bbox.min(axis) < colliders_b[pb].bbox.min(axis)) {
      sap_sorted_collision(colliders_a[pa], colliders_b, proxies_b + b,
                           proxy_num_b - b, axis, filter, cache);
      a++;
    } else {
      sap_sorted_collision(colliders_b[pb], colliders_a, proxies_a + a,
                           proxy_num_a - a, axis, filter, cache);
      b++;
    }
  }
}

/** Node of the broad-phase KD-tree. */
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

// for tree tree collision test
struct TTPair {
  KDNode* na = nullptr;
  bool exclude_na_children = false;
  KDNode* nb = nullptr;
  bool exclude_nb_children = false;
};

template <typename C>
/**
 * KDTree broad-phase accelerator for colliders of type `C`.
 *
 * Usage
 * - Construct, `init(colliders)`, then each frame update colliders' AABBs and
 *   call `update(world_bbox)` followed by `test_self_collision(...)` or
 *   `test_tree_collision(...)`.
 * - The tree maintains an in-order proxy array to avoid per-object relocation;
 *   internal operations work by shifting window boundaries and using delayed
 *   offsets to propagate edits efficiently.
 */
class KDTree {
 private:
  static constexpr int NODE_PROXY_NUM_THRESHOLD = 1024;

  int collider_num_ = 0;
  std::vector<C> colliders_;

  KDNode* root_ = nullptr;
  std::vector<KDNode*> stack_;  // for tree traversal
  std::vector<int> proxies_;    // in-order layout proxy array
  std::vector<int> buffer_;     // for both proxies and external colliders
  std::vector<CollisionCache<C>> local_cache_;  // thread local collision cache

 public:
  KDTree() = default;

  KDTree(const KDTree&) = delete;

  KDTree(KDTree&& tree) noexcept {
    collider_num_ = tree.collider_num_;
    colliders_ = std::move(tree.colliders_);
    root_ = tree.root_;
    stack_ = std::move(tree.stack_);
    proxies_ = std::move(tree.proxies_);
    buffer_ = std::move(tree.buffer_);
    local_cache_ = std::move(tree.local_cache_);

    tree.collider_num_ = 0;
    tree.root_ = nullptr;
  }

  ~KDTree() { delete_subtree(root_); }

  KDTree& operator=(const KDTree&) = delete;

  KDTree& operator=(KDTree&& tree) noexcept {
    collider_num_ = tree.collider_num_;
    colliders_ = std::move(tree.colliders_);
    root_ = tree.root_;
    stack_ = std::move(tree.stack_);
    proxies_ = std::move(tree.proxies_);
    buffer_ = std::move(tree.buffer_);
    local_cache_ = std::move(tree.local_cache_);

    tree.collider_num_ = 0;
    tree.root_ = nullptr;

    return *this;
  }

  void init(std::vector<C> colliders) {
    assert(!colliders.empty());

    delete_subtree(root_);
    collider_num_ = colliders.size();
    colliders_ = colliders;
    proxies_.resize(collider_num_);
    for (int i = 0; i < collider_num_; ++i) {
      proxies_[i] = i;
    }
    buffer_.resize(collider_num_);
    root_ = new KDNode{};
    root_->proxy_start = 0;
    root_->proxy_end = collider_num_;
    root_->population = collider_num_;
    local_cache_.resize(omp_get_max_threads());
  }

  /** Mutable access to stored colliders (ownership kept by the tree). */
  std::vector<C>& get_colliders() { return colliders_; }

  /** Read-only access to stored colliders. */
  const std::vector<C>& get_colliders() const { return colliders_; }

  /**
   * Update KD-tree structure for the current frame.
   * - Precondition: `colliders_` AABBs are up-to-date; `root_bbox` bounds the
   *   entire scene.
   * - Effect: lifts out-of-bounds proxies up, then applies split/collapse/
   *   evaluate/translate to approach an arrangement close to ideal without
   *   rebuilding from scratch.
   */
  void update(const Bbox& root_bbox) {
    assert(root_);

    root_->bbox = root_bbox;
    lift_unfit_up();
    optimize_structure();
  }

  /**
   * Enumerate potentially colliding pairs within this tree.
   * - Uses SAP per node on the axis of largest variance; schedules node work as
   *   OpenMP tasks and merges per-thread caches into `cache`.
   * - `filter` is applied before AABB checks and can skip domain-excluded
   *   pairs (e.g. static-static in incremental mode).
   */
  void test_self_collision(CollisionFilter<C> filter,
                           CollisionCache<C>& cache) {
    assert(root_);

    stack_.clear();
    buffer_.clear();
    for (int i = 0; i < local_cache_.size(); ++i) {
      local_cache_[i].clear();
    }

    root_->ext_start = 0;
    root_->ext_end = 0;
    test_node_collision(root_, filter);

    if (!root_->is_leaf()) {
      stack_.push_back(root_->right);
      stack_.push_back(root_->left);
    }

    // one main thread traverse the tree while collision detection at each node
    // is processed in parallel
#pragma omp parallel
#pragma omp single
    while (!stack_.empty()) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      update_ext_collider(n);
      test_node_collision(n, filter);

      // recurse into subtree
      if (!n->is_leaf()) {
        stack_.push_back(n->right);
        stack_.push_back(n->left);
      }
    }

    for (int i = 0; i < local_cache_.size(); ++i) {
      cache.insert(cache.end(), local_cache_[i].begin(), local_cache_[i].end());
    }
  }

  /**
   * Enumerate potentially colliding pairs between two KD-trees.
   * Traverses pairs of nodes and applies bipartite SAP when their bounds
   * intersect; honors `filter` identically to `test_self_collision`.
   */
  static void test_tree_collision(KDTree& ta, KDTree& tb,
                                  CollisionFilter<C> filter,
                                  CollisionCache<C>& cache) {
    assert(ta.root_ && tb.root_);

    std::vector<TTPair> tt_stack_;
    tt_stack_.push_back(TTPair{ta.root_, false, tb.root_, false});

    for (int i = 0; i < tt_stack_.size(); ++i) {
      auto [na, exclude_na_children, nb, exclude_nb_children] = tt_stack_[i];

      if (na == nullptr || nb == nullptr) {
        continue;
      }

      // test colliders on node itself only
      if (exclude_na_children && exclude_nb_children) {
        if (na->proxy_num() == 0 || nb->proxy_num() == 0) {
          continue;
        }

        Bbox& ba = na->is_leaf() ? na->bbox : na->plane_bbox;
        Bbox& bb = nb->is_leaf() ? nb->bbox : nb->plane_bbox;
        if (Bbox::is_colliding(ba, bb)) {
          int* start_a = ta.proxies_.data() + na->proxy_start;
          int num_a = na->proxy_num();
          int* start_b = tb.proxies_.data() + nb->proxy_start;
          int num_b = nb->proxy_num();

          int axis = sap_optimal_axis(ta.colliders_, start_a, num_a,
                                      tb.colliders_, start_b, num_b);
          sap_sort_proxies(ta.colliders_, start_a, num_a, axis);
          sap_sort_proxies(tb.colliders_, start_b, num_b, axis);
          sap_sorted_group_group_collision(ta.colliders_, start_a, num_a,
                                           tb.colliders_, start_b, num_b, axis,
                                           filter, cache);
        }

        continue;
      }

      if (exclude_na_children) {
        Bbox& ba = na->is_leaf() ? na->bbox : na->plane_bbox;
        if (Bbox::is_colliding(ba, nb->bbox)) {
          tt_stack_.push_back(TTPair{na, true, nb, true});
          tt_stack_.push_back(TTPair{na, true, nb->left, false});
          tt_stack_.push_back(TTPair{na, true, nb->right, false});
        }
        continue;
      }

      if (exclude_nb_children) {
        Bbox& bb = nb->is_leaf() ? nb->bbox : nb->plane_bbox;
        if (Bbox::is_colliding(na->bbox, bb)) {
          tt_stack_.push_back(TTPair{na, true, nb, true});
          tt_stack_.push_back(TTPair{na->left, false, nb, true});
          tt_stack_.push_back(TTPair{na->right, false, nb, true});
        }
        continue;
      }

      if (Bbox::is_colliding(na->bbox, nb->bbox)) {
        tt_stack_.push_back(TTPair{na, true, nb, true});
        tt_stack_.push_back(TTPair{na, true, nb->left, false});
        tt_stack_.push_back(TTPair{na, true, nb->right, false});
        tt_stack_.push_back(TTPair{na->left, false, nb, true});
        tt_stack_.push_back(TTPair{na->right, false, nb, true});
        tt_stack_.push_back(TTPair{na->left, false, nb->left, false});
        tt_stack_.push_back(TTPair{na->left, false, nb->right, false});
        tt_stack_.push_back(TTPair{na->right, false, nb->left, false});
        tt_stack_.push_back(TTPair{na->right, false, nb->right, false});
      }
    }
  }

  /** Clear scratch buffers and per-thread caches (structure remains intact). */
  void delete_cache() {
    stack_ = {};
    buffer_ = {};
    for (auto& cache : local_cache_) {
      cache = {};
    }
  }

 private:
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

    auto [mean, var] = proxy_mean_variance(
        colliders_, proxies_.data() + n->proxy_start, n->proxy_num());
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
        left_num++;
      }
      // strictly right
      else if (colliders_[p].bbox.min(n->axis) > n->position) {
        right_num++;
      }
      // on the plane
      else {
        middle_num++;
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

  // Propagate parent externals and parent node proxies into the child’s
  // external list using the parent plane. Buffer grows monotonically along the
  // traversal; child lists reference disjoint slices of `buffer_`.
  void update_ext_collider(KDNode* n) {
    assert(n->parent);

    // ensure buffer capacity
    int max_ext_num = n->parent->ext_num() + n->parent->proxy_num();
    int required_buffer_size = n->parent->ext_end + max_ext_num;
    ensure_buffer_size(required_buffer_size);

    n->ext_start = n->parent->ext_end;
    n->ext_end = n->parent->ext_end;
    int axis = n->parent->axis;
    float position = n->parent->position;

    // check parent's external colliders
    for (int i = n->parent->ext_start; i < n->parent->ext_end; ++i) {
      int p = buffer_[i];

      if (n->is_left()) {
        if (colliders_[p].bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      } else {
        if (colliders_[p].bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      }
    }

    // check parent node colliders
    for (int i = n->parent->proxy_start; i < n->parent->proxy_end; ++i) {
      int p = proxies_[i];

      if (n->is_left()) {
        if (colliders_[p].bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      } else {
        if (colliders_[p].bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      }
    }
  }

  // Run SAP at a node. If no externals, do intra-node only; otherwise also do
  // bipartite with externals. Work is spawned as an OpenMP task.
  void test_node_collision(KDNode* n, CollisionFilter<C> filter) {
    int* proxy_start = proxies_.data() + n->proxy_start;
    int proxy_num = n->proxy_num();
    if (proxy_num == 0) {
      return;
    }

    if (n->ext_num() == 0) {
      // node-node test
      int axis = sap_optimal_axis(colliders_, proxy_start, proxy_num);
      sap_sort_proxies(colliders_, proxy_start, proxy_num, axis);

#pragma omp task firstprivate(proxy_start, proxy_num, axis)
      {
        auto& cache = local_cache_[omp_get_thread_num()];
        sap_sorted_group_self_collision(colliders_, proxy_start, proxy_num,
                                        axis, filter, cache);
      }

    } else {
      // node-node and node-external test
      int* ext_start = buffer_.data() + n->ext_start;
      int ext_num = n->ext_num();
      // most collider resides on leaf, so no need to test external collider
      int axis = sap_optimal_axis(colliders_, proxy_start, proxy_num);
      sap_sort_proxies(colliders_, proxy_start, proxy_num, axis);
      sap_sort_proxies(colliders_, ext_start, ext_num, axis);

      // buffer will be overwrited once main thread traverse to another branch.
      // hence external collider buffer needs to be copied.
      std::vector<int> ext_copy(ext_start, ext_start + ext_num);

#pragma omp task firstprivate(proxy_start, proxy_num, axis, ext_copy)
      {
        auto& cache = local_cache_[omp_get_thread_num()];
        sap_sorted_group_self_collision(colliders_, proxy_start, proxy_num,
                                        axis, filter, cache);
        sap_sorted_group_group_collision(colliders_, proxy_start, proxy_num,
                                         colliders_, ext_copy.data(), ext_num,
                                         axis, filter, cache);
      }
    }
  }
};

}  // namespace silk
