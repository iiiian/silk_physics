#pragma once

#include <pdqsort.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <vector>

#include "bbox.hpp"

namespace silk {

template <typename C>
using CollisionCache = std::vector<std::pair<C*, C*>>;

template <typename C>
using CollisionFilterCallback = std::function<bool(const C&, const C&)>;

template <typename C>
std::pair<Eigen::Vector3f, Eigen::Vector3f> proxy_mean_variance(
    C* const* proxies, int proxy_num) {
  assert((proxy_num > 0));

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Vector3f variance = Eigen::Vector3f::Zero();
  for (int i = 0; i < proxy_num; ++i) {
    Eigen::Vector3f center = proxies[i]->bbox.center();
    mean += center;
    variance += center.cwiseAbs2();
  }
  mean /= proxy_num;
  variance = variance / proxy_num - mean.cwiseAbs2();

  return std::make_pair(std::move(mean), std::move(variance));
}

template <typename C>
int sap_optimal_axis(C* const* proxies, int proxy_num) {
  assert((proxy_num > 0));

  auto [mean, var] = proxy_mean_variance(proxies, proxy_num);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

template <typename C>
int sap_optimal_axis(C* const* proxies_a, int proxy_num_a, C* const* proxies_b,
                     int proxy_num_b) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  auto [mean_a, var_a] = proxy_mean_variance(proxies_a, proxy_num_a);
  auto [mean_b, var_b] = proxy_mean_variance(proxies_b, proxy_num_b);
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
void sap_sort_proxies(C** proxies, int proxy_num, int axis) {
  assert((proxy_num != 0));

  auto comp = [axis](C* a, C* b) -> bool {
    return (a->bbox.min(axis) < b->bbox.min(axis));
  };
  pdqsort_branchless(proxies, proxies + proxy_num, comp);
}

template <typename C>
void sap_sorted_collision(C* p1, C* const* proxies, int proxy_num, int axis,
                          CollisionFilterCallback<C> filter,
                          CollisionCache<C>& cache) {
  assert((proxy_num != 0));

  if (p1->bbox.max(axis) < proxies[0]->bbox.min(axis)) {
    return;
  }

  for (int i = 0; i < proxy_num; ++i) {
    C* p2 = proxies[i];
    // axis test
    if (p1->bbox.max(axis) < p2->bbox.min(axis)) {
      break;
    }
    // user provided collision filter
    if (!filter(*p1, *p2)) {
      continue;
    }

    if (Bbox::is_colliding(p1->bbox, p2->bbox)) {
      cache.emplace_back(p1, p2);
    }
  }
}

template <typename C>
void sap_sorted_group_self_collision(C* const* proxies, int proxy_num, int axis,
                                     CollisionFilterCallback<C> filter,
                                     CollisionCache<C>& cache) {
  assert((proxy_num > 0));

  for (int i = 0; i < proxy_num - 1; ++i) {
    sap_sorted_collision(proxies[i], proxies + i + 1, proxy_num - i - 1, axis,
                         filter, cache);
  }
}

template <typename C>
void sap_sorted_group_group_collision(C* const* proxies_a, int proxy_num_a,
                                      C* const* proxies_b, int proxy_num_b,
                                      int axis,
                                      CollisionFilterCallback<C> filter,
                                      CollisionCache<C>& cache) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    C* pa = proxies_a[a];
    C* pb = proxies_b[b];

    if (pa->bbox.min(axis) < pb->bbox.min(axis)) {
      sap_sorted_collision(pa, proxies_b + b, proxy_num_b - b, axis, filter,
                           cache);
      a++;
    } else {
      sap_sorted_collision(pb, proxies_a + a, proxy_num_a - a, axis, filter,
                           cache);
      b++;
    }
  }
}

struct KDNode {
  KDNode* parent = nullptr;
  KDNode* left = nullptr;
  KDNode* right = nullptr;

  Bbox bbox = {};
  Bbox plane_bbox = {};
  int axis = 0;             // split plane axis
  float position = 0.0f;    // split plane position
  int proxy_start = 0;      // proxies array start
  int proxy_end = 0;        // proxies array end
  int population = 0;       // subtree proxy num
  int delay_offset = 0;     // delayed update to proxy start/end
  int ext_start = 0;        // external collider buffer start
  int ext_end = 0;          // external collider buffer end
  uint32_t generation = 0;  // for tree-tree collision

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
  static constexpr int NODE_PROXY_NUM_THRESHOLD = 1024;

  int collider_num_ = 0;
  C* colliders_ = nullptr;

  KDNode* root_ = nullptr;
  std::vector<KDNode*> stack_;  // for tree traversal
  std::vector<C*> proxies_;     // in-order layout proxy array
  std::vector<C*> buffer_;      // for both proxies and external colliders
  std::vector<CollisionCache<C>> local_cache_;  // thread local collision cache

 public:
  KDTree() = default;

  KDTree(KDTree&) = delete;

  KDTree(KDTree&& tree) noexcept {
    collider_num_ = tree.collider_num_;
    colliders_ = tree.colliders_;
    root_ = tree.root_;
    stack_ = std::move(tree.stack_);
    proxies_ = std::move(tree.proxies_);
    buffer_ = std::move(tree.buffer_);
    local_cache_ = std::move(tree.local_cache_);

    tree.collider_num_ = 0;
    tree.colliders_ = nullptr;
    tree.root_ = nullptr;
  }

  ~KDTree() { delete_subtree(root_); }

  KDTree& operator=(KDTree&) = delete;

  KDTree& operator=(KDTree&& tree) noexcept {
    collider_num_ = tree.collider_num_;
    colliders_ = tree.colliders_;
    root_ = tree.root_;
    stack_ = std::move(tree.stack_);
    proxies_ = std::move(tree.proxies_);
    buffer_ = std::move(tree.buffer_);
    local_cache_ = std::move(tree.local_cache_);

    tree.collider_num_ = 0;
    tree.colliders_ = nullptr;
    tree.root_ = nullptr;

    return *this;
  }

  void init(C* colliders, int collider_num) {
    assert(colliders);
    assert((collider_num > 0));

    delete_subtree(root_);
    collider_num_ = collider_num;
    colliders_ = colliders;
    proxies_.resize(collider_num);
    buffer_.resize(collider_num);
    for (int i = 0; i < collider_num_; ++i) {
      proxies_[i] = colliders + i;
    }
    root_ = new KDNode{};
    root_->proxy_start = 0;
    root_->proxy_end = collider_num;
    root_->population = collider_num;
    local_cache_.resize(omp_get_max_threads());
  }

  void update(const Bbox& root_bbox) {
    assert(root_);

    root_->bbox = root_bbox;
    lift_unfit_up();
    optimize_structure();
  }

  void test_self_collision(CollisionFilterCallback<C> filter,
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

  static void test_tree_collision(KDTree& ta, KDTree& tb,
                                  CollisionFilterCallback<C> filter,
                                  CollisionCache<C>& cache) {
    assert(ta.root_ && tb.root_);

    // this should never happen in normal scenario
    if (ta.root_->generation == std::numeric_limits<uint32_t>::max()) {
      ta.reset_generation();
    }
    if (tb.root_->generation == std::numeric_limits<uint32_t>::max()) {
      tb.reset_generation();
    }

    // since root is guaranteed to be visited during tree-tree collision test,
    // the generation of root node is the last generation of tree. The current
    // generation is last generation plus one
    uint32_t gen_a = ta.root_->generation + 1;
    uint32_t gen_b = tb.root_->generation + 1;

    std::vector<std::pair<KDNode*, KDNode*>> pair_stack_;
    pair_stack_.emplace_back(ta.root_, tb.root_);

    for (int i = 0; i < pair_stack_.size(); ++i) {
      auto [na, nb] = pair_stack_[i];
      // pair_stack_.pop_back();

      if (!na || !nb) {
        continue;
      }

      // the node has never been visited if generation doesn't match
      bool is_na_new = na->generation != gen_a;
      bool is_nb_new = nb->generation != gen_b;
      na->generation = gen_a;
      nb->generation = gen_b;

      if (is_na_new && is_nb_new) {
        if (Bbox::is_colliding(na->bbox, nb->bbox)) {
          pair_stack_.emplace_back(na->left, nb->left);
          pair_stack_.emplace_back(na->left, nb->right);
          pair_stack_.emplace_back(na->right, nb->left);
          pair_stack_.emplace_back(na->right, nb->right);
          pair_stack_.emplace_back(na, nb->left);
          pair_stack_.emplace_back(na, nb->right);
          pair_stack_.emplace_back(na->left, nb);
          pair_stack_.emplace_back(na->right, nb);
          pair_stack_.emplace_back(na, nb);
        }
        continue;
      }

      if (is_na_new) {
        if (Bbox::is_colliding(na->bbox, nb->plane_bbox)) {
          pair_stack_.emplace_back(na->left, nb);
          pair_stack_.emplace_back(na->right, nb);
          pair_stack_.emplace_back(na, nb);
        }
        continue;
      }

      if (is_nb_new) {
        if (Bbox::is_colliding(na->plane_bbox, nb->bbox)) {
          pair_stack_.emplace_back(na, nb->left);
          pair_stack_.emplace_back(na, nb->right);
          pair_stack_.emplace_back(na, nb);
        }
        continue;
      }

      // both node a and node b is old, test on plane proxy only
      if (na->proxy_num() == 0 || nb->proxy_num() == 0) {
        continue;
      }
      Bbox& ba = (na->is_leaf()) ? na->bbox : na->plane_bbox;
      Bbox& bb = (nb->is_leaf()) ? nb->bbox : nb->plane_bbox;
      if (Bbox::is_colliding(ba, bb)) {
        C** start_a = ta.proxies_.data() + na->proxy_start;
        int num_a = na->proxy_num();
        C** start_b = tb.proxies_.data() + nb->proxy_start;
        int num_b = nb->proxy_num();

        int axis = sap_optimal_axis(start_a, num_a, start_b, num_b);
        sap_sort_proxies(start_a, num_a, axis);
        sap_sort_proxies(start_b, num_b, axis);
        // auto cache = ta.local_cache_[omp_get_thread_num()];
        sap_sorted_group_group_collision(start_a, num_a, start_b, num_b, axis,
                                         filter, cache);
      }
    }
  }

  void delete_cache() {
    stack_ = {};
    buffer_ = {};
    for (auto& cache : local_cache_) {
      local_cache_ = {};
    }
  }

 private:
  void ensure_buffer_size(int num) {
    if (buffer_.size() < num) {
      buffer_.resize(num);
    }
  }

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
  // plane axis should have the max variance and the plane position is the
  // mean.
  void find_optimal_plane(KDNode* n) const {
    assert((n->proxy_num() > 0));

    auto [mean, var] =
        proxy_mean_variance(proxies_.data() + n->proxy_start, n->proxy_num());
    var.maxCoeff(&n->axis);
    n->position = mean(n->axis);
  }

  int partition_unfit_proxy_left(const KDNode* n) {
    auto is_outside = [n](C* p) -> bool {
      return !(n->bbox.is_inside(p->bbox));
    };

    C** start = proxies_.data() + n->proxy_start;
    C** end = proxies_.data() + n->proxy_end;
    C** new_start = std::partition(start, end, is_outside);

    return new_start - start;
  }

  int partition_unfit_proxy_right(const KDNode* n) {
    auto is_inside = [n](C* p) -> bool { return n->bbox.is_inside(p->bbox); };

    C** start = proxies_.data() + n->proxy_start;
    C** end = proxies_.data() + n->proxy_end;
    C** new_end = std::partition(start, end, is_inside);

    return end - new_end;
  }

  void shift_proxy_left(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(C*);
    size_t shift_size = shift_num * sizeof(C*);
    C** left = proxies_.data() + proxy_start - shift_num;
    C** right = proxies_.data() + proxy_start;
    ensure_buffer_size(proxy_num);

    // copy right chunk to temp buffer
    std::memcpy(buffer_.data(), right, copy_size);
    // shift left chunk right
    std::memmove(left + proxy_num, left, shift_size);
    // copy right chunk back
    std::memcpy(left, buffer_.data(), copy_size);
  }

  void shift_proxy_right(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(C*);
    size_t shift_size = shift_num * sizeof(C*);
    C** left = proxies_.data() + proxy_start;
    C** right = proxies_.data() + proxy_start + proxy_num;
    ensure_buffer_size(proxy_num);

    // copy left chunk to temp buffer
    std::memcpy(buffer_.data(), left, copy_size);
    // shift right chunk left
    std::memmove(left, right, shift_size);
    // copy left chunk back
    std::memcpy(left + shift_num, buffer_.data(), copy_size);
  }

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
          assert((n->proxy_end < collider_num_));
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
      C* p = proxies_[i];

      // strictly left
      if (p->bbox.max(n->axis) < n->position) {
        std::swap(proxies_[left_end], proxies_[i]);
        ++left_end;
        ++i;
      }
      // strictly right
      else if (p->bbox.min(n->axis) > n->position) {
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
    assert((n->left->proxy_end + n->left->delay_offset < collider_num_));
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

  static void set_children_bbox(KDNode* n) {
    assert((n->left && n->right));

    n->left->bbox = n->bbox;
    n->left->bbox.max(n->axis) = n->position;
    n->right->bbox = n->bbox;
    n->right->bbox.min(n->axis) = n->position;
  }

  // find optimal split plane then split the leaf
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
      C* p = proxies_[i];
      // strictly left
      if (p->bbox.max(n->axis) < n->position) {
        left_num++;
      }
      // strictly right
      else if (p->bbox.min(n->axis) > n->position) {
        right_num++;
      }
      // on the plane
      else {
        middle_num++;
      }
    }

    left_num += n->left->population;
    right_num += n->right->population;

    int num = n->proxy_num();
    float t_min = 0.5f * float(num) * (0.5f * float(num) - 1.0f);
    float t_max = 0.5f * float(num) * (float(num) - 1.0f);
    float t = 0.5f * float((left_num * left_num + middle_num * middle_num +
                            right_num * right_num - num) +
                           middle_num * (left_num + right_num));
    float cost = (t - t_min) / (t_max - t_min);
    float balance = float(std::min(left_num, right_num)) /
                    float((middle_num + std::max(left_num, right_num)));
    return (cost <= balance);
  }

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

  void translate(KDNode* n) {
    assert((n->proxy_num() != 0));

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      mean += proxies_[i]->bbox.center();
    }

    mean /= float(n->proxy_num());
    n->position = mean(n->axis);
  }

  void erase(KDNode* n, bool keep_left) {
    assert((n->left && n->right));
    // erase only happens after translate, which will lift all proxies up to n
    assert((n->left->population == 0));
    assert((n->right->population == 0));

    KDNode* main;
    KDNode* other;
    if (keep_left) {
      main = n->left;
      other = n->right;
    } else {
      main = n->right;
      other = n->left;
    }

    n->left = main->left;
    n->right = main->right;
    n->axis = main->axis;
    n->position = main->position;

    if (!main->is_leaf()) {
      main->left->parent = n;
      main->right->parent = n;
    }

    delete_subtree(other);
    delete main;
  }

  void set_plane_bbox(KDNode* n) {
    if (n->proxy_num() == 0) {
      return;
    }
    n->plane_bbox = proxies_[n->proxy_start]->bbox;
    for (int i = n->proxy_start + 1; i < n->proxy_end; ++i) {
      n->plane_bbox.merge_inplace(proxies_[i]->bbox);
    }
  }

  void optimize_structure() {
    stack_.clear();

    // pre-order traverse the tree
    stack_.push_back(root_);
    while (!stack_.empty()) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      // propagate and apply delay offset
      if (n->delay_offset == 0) {
        return;
      }

      if (!n->is_leaf()) {
        n->left->delay_offset += n->delay_offset;
        n->right->delay_offset += n->delay_offset;
      }
      n->proxy_start += n->delay_offset;
      assert((n->proxy_start >= 0));
      n->proxy_end += n->delay_offset;
      assert((n->proxy_end <= collider_num_));
      n->delay_offset = 0;

      // split a leaf if proxy num is too large
      if (n->is_leaf()) {
        if (n->proxy_num() > NODE_PROXY_NUM_THRESHOLD) {
          split_leaf(n);
          stack_.push_back(n->right);
          stack_.push_back(n->left);
        }
        n->generation = 0;
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
        n->generation = 0;
        stack_.push_back(n->right);
        stack_.push_back(n->left);
        continue;
      }

      // original plane is not optimal, try translating the plane
      int left_num = n->left->population;
      int right_num = n->right->population;
      lift_subtree(n);
      translate(n);
      if (evaluate(n)) {
        filter(n);
        set_children_bbox(n);
        set_plane_bbox(n);
        n->generation = 0;
        stack_.push_back(n->right);
        stack_.push_back(n->left);
        continue;
      }

      // translated plane is still not optimal, erase the node
      erase(n, (left_num > right_num));
      stack_.push_back(n);
    }
  }

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
      C* p = buffer_[i];

      if (n->is_left()) {
        if (p->bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      } else {
        if (p->bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      }
    }

    // check parent node colliders
    for (int i = n->parent->proxy_start; i < n->parent->proxy_end; ++i) {
      C* p = proxies_[i];

      if (n->is_left()) {
        if (p->bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      } else {
        if (p->bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
        }
      }
    }
  }

  void test_node_collision(KDNode* n, CollisionFilterCallback<C> filter) {
    C** proxy_start = proxies_.data() + n->proxy_start;
    int proxy_num = n->proxy_num();
    if (proxy_num == 0) {
      return;
    }

    if (n->ext_num() == 0) {
      // node-node test
      int axis = sap_optimal_axis(proxy_start, proxy_num);
      sap_sort_proxies(proxy_start, proxy_num, axis);

#pragma omp task firstprivate(proxy_start, proxy_num, axis)
      {
        auto& cache = local_cache_[omp_get_thread_num()];
        sap_sorted_group_self_collision(proxy_start, proxy_num, axis, filter,
                                        cache);
      }
    } else {
      // node-node and node-external test
      C** ext_start = buffer_.data() + n->ext_start;
      int ext_num = n->ext_num();
      // most collider resides on leaf, so no need to test external collider
      int axis = sap_optimal_axis(proxy_start, proxy_num);
      sap_sort_proxies(proxy_start, proxy_num, axis);
      sap_sort_proxies(ext_start, ext_num, axis);

      // buffer will be overwrited once main thread traverse to another branch.
      // hence external collider buffer needs to be copied.
      std::vector<C*> ext_copy(ext_start, ext_start + ext_num);
#pragma omp task firstprivate(proxy_start, proxy_num, axis, ext_copy)
      {
        auto& cache = local_cache_[omp_get_thread_num()];
        sap_sorted_group_self_collision(proxy_start, proxy_num, axis, filter,
                                        cache);
        sap_sorted_group_group_collision(proxy_start, proxy_num,
                                         ext_copy.data(), ext_num, axis, filter,
                                         cache);
      }
    }
  }

  void reset_generation() {
    stack_.clear();

    stack_.push_back(root_);
    while (!stack_.empty()) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      n->generation = 0;

      // recurse into subtree
      if (!n->is_leaf()) {
        stack_.push_back(n->right);
        stack_.push_back(n->left);
      }
    }
  }
};

}  // namespace silk
