#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <iterator>
#include <random>
#include <vector>

#include "bbox.hpp"
#include "collision_helper.hpp"
#include "sap.hpp"

namespace silk {

struct KDNode {
  KDNode* parent;
  KDNode* left;
  KDNode* right;

  Bbox bbox;
  int axis;               // split plane axis
  float position;         // split plane position
  int proxy_start;        // proxies array start
  int proxy_end;          // proxies array end
  int ext_start;          // external collider buffer start
  int ext_end;            // external collider buffer end
  int static_num;         // static node proxy num
  int population;         // subtree proxy num
  int static_population;  // static subtree proxy num
  int delay_offset;       // delayed update to proxy start/end
  bool is_ext_static;     // is external colliders all static

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

  bool is_static() const { return (static_num == proxy_num()); }

  bool is_subtree_static() const { return (static_population == population); }
};

template <typename T>
class KDTree {
 public:
  KDTree(std::vector<BboxCollider<T>>* p_colliders) {
    assert(p_colliders);

    collider_num_ = p_colliders->size;
    root_ = new KDNode{.parent = nullptr,
                       .left = nullptr,
                       .right = nullptr,
                       .bbox = Bbox::make_inf_bbox(),
                       .axis = 0,
                       .position = 0,
                       .proxy_start = 0,
                       .proxy_end = collider_num_,
                       .ext_start = 0,
                       .ext_end = 0,
                       .static_num = 0,
                       .population = collider_num_,
                       .static_population = 0,
                       .delay_offset = 0,
                       .is_ext_static = false};
    proxies_.resize(collider_num_);
    for (int i = 0; i < collider_num_; ++i) {
      proxies_[i] = p_colliders->data() + i;
    }
    buffer_.resize(collider_num_);
  }

  ~KDTree() { delete_subtree(root_); }

  void update() {
    refit();
    optimize();
    update_static_population();
  }

  void test_self_collision(CollisionFilterCallback<T> filter_callback,
                           CollisionCache<T>& cache) {
    root_->ext_start = 0;
    root_->ext_end = 0;
    test_node_collision(root_, filter_callback, cache);

    assert(stack_.empty());

    if (!root_->is_leaf()) {
      stack_.push_back(root_->right);
      stack_.push_back(root_->left);
    }

    while (stack_.size() != 0) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      update_ext_collider(n);
      test_node_collision(n, filter_callback, cache);

      // recurse into subtree, aviod static-static pair
      if (!n->is_leaf()) {
        if (!(n->right->is_subtree_static() && n->is_ext_static)) {
          stack_.push_back(n->right);
        }

        if (!(n->left->is_subtree_static() && n->is_ext_static)) {
          stack_.push_back(n->left);
        }
      }
    }
  }

  static void test_tree_collision(KDTree& ta, KDTree& tb,
                                  CollisionFilterCallback<T> filter_callback,
                                  CollisionCache<T>& cache) {
    std::vector<std::pair<KDNode*, KDNode*>> pair_stack_;
    pair_stack_.push_back({ta.root_, tb.root_});
    while (!pair_stack_.empty()) {
      auto [na, nb] = pair_stack_.back();
      pair_stack_.pop_back();

      if (Bbox::is_disjoint(na->bbox, nb->bbox)) {
        continue;
      }

      if (na->is_subtree_static() && nb->is_subtree_static()) {
        continue;
      }

      if (!(na->is_static() && nb->is_static())) {
        BboxCollider<T>** start_a = ta.proxies_.data() + na->proxy_start;
        int num_a = na->proxy_num();
        BboxCollider<T>** start_b = tb.proxies_.data() + nb->proxy_start;
        int num_b = nb->proxy_num();
        int axis = sap_find_optimal_axis(start_a, num_a, start_b, num_b);
        sap_sort_proxies(start_a, num_a, axis);
        sap_sort_proxies(start_b, num_b, axis);
        sap_test_sorted_group_collision(start_a, num_a, start_b, num_b, axis,
                                        filter_callback, cache);
      }

      if (!na->is_leaf()) {
        pair_stack_.push_back({na->left, nb});
        pair_stack_.push_back({na->right, nb});
      }
      if (!nb->is_leaf()) {
        pair_stack_.push_back({na, nb->left});
        pair_stack_.push_back({na, nb->right});
      }
    }
  }

 private:
  static constexpr int NODE_PROXY_THRESHOLD = 512;
  static constexpr int APPROX_PLANE_SAMPLE_NUM = 16;
  static constexpr int OPTIMIAL_PLANE_SAMPLE_THRESHOLD = 32;

  int collider_num_;
  bool is_incremental_;

  KDNode* root_;
  std::vector<KDNode*> stack_;             // for tree traversal
  std::vector<BboxCollider<T>*> proxies_;  // in-order layout proxy array
  std::vector<BboxCollider<T>*>
      buffer_;  // for both proxies and external colliders

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

  void update_static_population() {
    // BFS stack fill
    stack_.push_back(root_);
    for (int i = 0; i < stack_.size(); ++i) {
      KDNode* n = stack_[i];
      if (!n->is_leaf()) {
        stack_.push_back(n->left);
        stack_.push_back(n->right);
      }
    }

    // bottom up refit, push unfit node up.
    for (int i = stack_.size() - 1; i >= 0; i--) {
      KDNode* n = stack_.back();

      n->static_num = 0;
      for (int i = n->proxy_start; i < n->proxy_end; ++i) {
        if (proxies_[i]->is_static) {
          n->static_num++;
        }
      }

      if (n->is_leaf()) {
        n->static_population = n->static_num;
      } else {
        n->static_population = n->static_num + n->left->static_population +
                               n->right->static_population;
      }
    }

    stack_.clear();
  }

  int push_unfit_right(KDNode* n) {
    auto is_inside = [n](BboxCollider<T>* obj_proxy) -> bool {
      return n->bbox.is_inside(obj_proxy->bbox);
    };

    auto start_iter = std::advance(proxies_.begin(), n->proxy_start);
    auto end_iter = std::advance(proxies_.begin(), n->proxy_end);
    auto new_end_iter = std::partition(start_iter, end_iter, is_inside);
    n->proxy_end = std::distance(proxies_.begin(), new_end_iter);

    return std::distance(new_end_iter, end_iter);
  }

  int push_unfit_left(KDNode* n) {
    auto is_outside = [n](BboxCollider<T>* obj_proxy) -> bool {
      return !(n->bbox.is_inside(obj_proxy->bbox));
    };

    auto start_iter = std::advance(proxies_.begin(), n->proxy_start);
    auto end_iter = std::advance(proxies_.begin(), n->proxy_end);
    auto new_start_iter = std::partition(start_iter, end_iter, is_outside);
    n->proxy_start = std::distance(proxies_.begin(), new_start_iter);

    return std::distance(start_iter, new_start_iter);
  }

  void shift_proxy_right(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(BboxCollider<T>*);
    size_t shift_size = shift_num * sizeof(BboxCollider<T>*);
    BboxCollider<T>* left = proxies_.data() + proxy_start;
    BboxCollider<T>* right = proxies_.data() + proxy_start + proxy_num;
    buffer_.reserve(proxy_num);

    // copy left chunk to temp buffer
    std::memcpy(buffer_.data(), left, copy_size);
    // shift right chunk left
    std::memmove(left, right, shift_size);
    // copy left chunk to right
    std::memcpy(left + shift_num, buffer_.data(), copy_size);
  }

  void shift_proxy_left(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(BboxCollider<T>*);
    size_t shift_size = shift_num * sizeof(BboxCollider<T>*);
    BboxCollider<T>* left = proxies_.data() + proxy_start - shift_num;
    BboxCollider<T>* right = proxies_.data() + proxy_start;
    buffer_.reserve(copy_size);

    // copy right chunk to temp buffer
    std::memcpy(buffer_.data(), right, copy_size);
    // shift left chunk right
    std::memmove(left + proxy_num, left, shift_size);
    // copy right chunk to left
    std::memcpy(left, buffer_.data(), copy_size);
  }

  // split internal node based on plane
  void split_internal_node(KDNode* n) {
    assert((n->left || n->right));
    assert((n->proxy_end - n->proxy_start > 0));

    // partition object proxies by plane.
    // strictly left  -> move to left partition
    // on the plane   -> move to middle partition
    // strictly right -> move to right partition
    int left_end = n->proxy_start;
    int right_start = n->proxy_end;
    for (int i = n->proxy_start; i < right_start;) {
      BboxCollider<T>* o = proxies_[i];
      // strictly left
      if (o->bbox.max(n->axis) < n->position) {
        std::swap(proxies_[left_end], proxies_[i]);
        proxies_[left_end]->node = n->left;
        left_end++;
        i++;
      }
      // strictly right
      else if (o->bbox.min(n->axis) > n->position) {
        std::swap(proxies_[right_start], proxies_[i]);
        proxies_[right_start - 1]->node = n->right;
        right_start--;
      }
      // on the plane, do nothing
      else {
        i++;
      }
    }

    // move left partition down one node level
    int left_num = left_end - n->proxy_start;
    if (left_num != 0) {
      n->left->proxy_end += left_num;
      n->left->population += left_num;
      if (!n->left->is_leaf()) {
        shift_proxy_left(n->proxy_start, left_num, n->left->right->population);
        n->left->right->proxy_start += left_num;
        n->left->right->proxy_end += left_num;
      }
    }

    // move right partition down one node level
    int right_num = n->proxy_end - right_start;
    if (right_num != 0) {
      n->right->proxy_start -= right_num;
      n->right->population += right_num;
      if (!n->right->is_leaf()) {
        shift_proxy_right(right_start, right_num, n->right->left->population);
        n->right->left->proxy_start -= right_num;
        n->right->left->proxy_end -= right_num;
      }
    }
  }

  // find optimal split plane based on mean and variance of bbox center.
  // plane axis should has the max variance and the plane position is the
  // mean.
  void find_optimal_plane(KDNode* n, int& axis, float& position) const {
    auto [mean, variance] = find_proxy_mean_variance(
        proxies_.data() + n->proxy_start, n->proxy_num());
    variance.maxCoeff(&axis);
    position = mean(axis);
  }

  // find approximate split plane based on mean and variance of bbox center.
  // sample a few bbox only.
  void find_approx_plane(KDNode* n, int& axis, float& position) {
    static std::random_device rand_device;
    static std::mt19937 rand_generator(rand_device());

    if (n->proxy_num() <= OPTIMIAL_PLANE_SAMPLE_THRESHOLD) {
      find_optimal_plane(n, axis, position);
      return;
    }

    int start = n->proxy_start;
    int end = n->proxy_start + APPROX_PLANE_SAMPLE_NUM;
    for (int i = start; i < end; ++i) {
      std::uniform_int_distribution rand_dist(i, n->proxy_end - 1);
      int rand_idx = rand_dist(rand_generator);
      std::swap(proxies_[i], proxies_[rand_idx]);
    }

    auto [mean, variance] = find_proxy_mean_variance(proxies_.data() + start,
                                                     APPROX_PLANE_SAMPLE_NUM);
    variance.maxCoeff(&axis);
    position = mean(axis);
  }

  // find optimal split plane then split the leaf
  void split_leaf_node(KDNode* n) {
    assert(!(n->left || n->right));
    assert((n->proxy_end - n->proxy_start > 0));

    find_optimal_plane(n, n->axis, n->position);

    // create new children
    Bbox left_bbox = n->bbox;
    left_bbox.max(n->axis) = n->position;
    n->left = new KDNode{.parent = n,
                         .left = nullptr,
                         .right = nullptr,
                         .bbox = left_bbox,
                         .axis = 0,
                         .position = 0,
                         .proxy_start = n->proxy_start,
                         .proxy_end = n->proxy_start,
                         .ext_start = 0,
                         .ext_end = 0,
                         .static_num = 0,
                         .population = 0,
                         .static_population = 0,
                         .delay_offset = 0,
                         .is_ext_static = false};
    Bbox right_bbox = n->bbox;
    right_bbox.min(n->axis) = n->position;
    n->right = new KDNode{.parent = n,
                          .left = nullptr,
                          .right = nullptr,
                          .bbox = right_bbox,
                          .axis = 0,
                          .position = 0,
                          .proxy_start = n->proxy_end,
                          .proxy_end = n->proxy_end,
                          .ext_start = 0,
                          .ext_end = 0,
                          .static_num = 0,
                          .population = 0,
                          .static_population = 0,
                          .delay_offset = 0,
                          .is_ext_static = false};

    split_internal_node(n);
  }

  // count the number of objects on the left, middle, and right of the plane
  void count_left_middle_right(KDNode* n, int& left_num, int& middle_num,
                               int& right_num) const {
    left_num = 0;
    middle_num = 0;
    right_num = 0;

    // partition object proxies by plane.
    // strictly left  -> left partition
    // on the plane   -> middle partition
    // strictly right -> right partition
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      BboxCollider<T>* o = proxies_[i];
      // strictly left
      if (o->bbox.max(n->axis) < n->position) {
        left_num++;
      }
      // strictly right
      else if (o->bbox.min(n->axis) > n->position) {
        right_num++;
      }
      // on the plane
      else {
        middle_num++;
      }
    }
  }

  void lift_subtree(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    n->proxy_end += n->right->population;

    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      proxies_[i].node = n;
    }

    size_t init_stack_size = stack_.size();
    // lift left subtree
    stack_.push_back(n->left);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (current->population == 0) {
        continue;
      }

      if (!current->is_leaf()) {
        stack_.push_back(current->left);
        stack_.push_back(current->right);
      }

      current->proxy_start = n->proxy_start;
      current->proxy_end = n->proxy_start;
      current->population = 0;
    }
    // lift right subtree
    stack_.push_back(n->right);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (current->population == 0) {
        continue;
      }

      if (!current->is_leaf()) {
        stack_.push_back(current->left);
        stack_.push_back(current->right);
      }

      current->proxy_end = n->proxy_end;
      current->proxy_end = n->proxy_end;
      current->population = 0;
    }
  }

  void refit() {
    assert(stack_.empty());

    // BFS stack fill
    stack_.push_back(root_);
    for (int i = 0; i < stack_.size(); ++i) {
      KDNode* n = stack_[i];
      if (!n->is_leaf()) {
        stack_.push_back(n->left);
        stack_.push_back(n->right);
      }
    }

    // bottom up refit, push unfit node up. root is excluded
    for (int i = stack_.size() - 1; i > 0; i--) {
      KDNode* n = stack_[i];

      if (n->proxy_num() == 0) {
        continue;
      }

      if (n->is_leaf()) {
        if (n->is_left()) {
          // left leaf
          n->parent->proxy_start -= push_unfit_right(n);
          n->population = n->proxy_num();
        } else {
          // right leaf
          n->parent->proxy_end += push_unfit_left(n);
          n->population = n->proxy_num();
        }
      } else {
        if (n->is_left()) {
          // left internal
          int unfit_num = push_unfit_right(n);
          shift_proxy_right(n->proxy_end, unfit_num, n->right->population);
          n->proxy_end -= unfit_num;
          n->parent->proxy_start -= unfit_num;
          n->right->delay_offset = -unfit_num;
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
        } else {
          // right internal
          int unfit_num = push_unfit_left(n);
          shift_proxy_right(n->proxy_start, unfit_num, n->left->population);
          n->proxy_start += unfit_num;
          n->parent->proxy_end += unfit_num;
          n->left->delay_offset = +unfit_num;
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
        }
      }
    }

    if (root_->is_leaf()) {
      root_->population = proxies_.size();
    } else {
      root_->population = root_->proxy_num() + root_->left->population +
                          root_->right->population;
    }

    stack_.clear();
  }

  // collapse subtree into leaf
  void collapse(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    n->proxy_end += n->right->population;
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      proxies_[i]->node = n;
    }
    delete_subtree(n);
    n->left = nullptr;
    n->right = nullptr;
  }

  bool evaluate(float lp, float mp, float rp) const {
    float s = lp + mp + rp;
    float t_min = 0.5f * s * (0.5f * s - 1.0f);
    float t_max = 0.5f * s * (s - 1.0f);
    float t = 0.5 * (lp * lp + mp * mp + rp * rp - s) + mp * (lp + rp);
    float cost = (t - t_min) / (t_max - t_min);
    float balance = std::min(lp, rp) / (mp + std::max(lp, rp));
    return (cost <= balance);
  }

  void translate(KDNode* n) {
    assert((n->proxy_end > n->proxy_start));

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      mean += proxies_[i]->bbox.center();
    }

    mean /= float(n->proxy_end - n->proxy_start);
    n->position = mean(n->axis);
  }

  KDNode* erase(KDNode* n, bool keep_left_subtree) {
    assert((n->left && n->right));
    assert((n->left->population == 0));
    assert((n->right->population == 0));

    KDNode* main_subtree;
    KDNode* other_subtree;
    if (keep_left_subtree) {
      main_subtree = n->left;
      other_subtree = n->right;
    } else {
      main_subtree = n->right;
      other_subtree = n->left;
    }

    main_subtree->proxy_start = n->proxy_start;
    main_subtree->proxy_end = n->proxy_end;
    main_subtree->population = n->population;
    main_subtree->parent = n->parent;
    main_subtree->bbox = n->bbox;
    if (n->parent) {
      if (n == n->parent->left) {
        n->parent->left = main_subtree;
      } else {
        n->parent->right = main_subtree;
      }
    }

    delete n;
    delete_subtree(other_subtree);
  }

  void optimize() {
    assert(stack_.empty());

    // DFS fill stack
    stack_.push_back(root_);
    while (!stack_.empty()) {
      KDNode* n = stack_.back();
      stack_.pop_back();

      // propagate and apply delay offset
      n->left->delay_offset += n->delay_offset;
      n->right->delay_offset += n->delay_offset;
      n->proxy_start += n->delay_offset;
      n->proxy_end += n->delay_offset;
      n->delay_offset = 0;

      // if population of leaf is too large, split
      if (n->is_leaf()) {
        if (n->proxy_num() > NODE_PROXY_THRESHOLD) {
          split_leaf_node(n);
          stack_.push_back(n->right);
          stack_.push_back(n->left);
        }
        continue;
      }

      // if population of internal nodes is too small, collapse into leaf node
      if (n->population < NODE_PROXY_THRESHOLD) {
        collapse(n);
        stack_.push_back(n);
        continue;
      }

      // first try to reuse the original plane
      int left_num, middle_num, right_num;
      count_left_middle_right(n, left_num, middle_num, right_num);
      bool is_plane_optimal =
          evaluate(left_num + n->left->population, middle_num,
                   right_num + n->right->population);
      if (is_plane_optimal) {
        split_internal_node(n);
        stack_.push_back(n->left);
        stack_.push_back(n->right);
        continue;
      }

      // original plane is not optimal, try translating the plane
      lift_subtree(n);
      translate(n);
      count_left_middle_right(n, left_num, middle_num, right_num);
      is_plane_optimal = evaluate(left_num, middle_num, right_num);
      if (is_plane_optimal) {
        split_internal_node(n);
        stack_.push_back(n->left);
        stack_.push_back(n->right);
        continue;
      }

      // translated plane is still not optimal, erase the node
      stack_.push_back(erase(n, (left_num > right_num)));
    }
  }

  void update_ext_collider(KDNode* n) {
    assert(n);
    assert(n->parent);

    // resize buffer
    int max_ext_num = n->parent->ext_num() + n->parent->proxy_num();
    int required_buffer_size = n->parent->ext_end + max_ext_num;
    buffer_.resize(required_buffer_size);

    n->ext_start = n->parent->ext_end;
    n->ext_end = n->parent->ext_end;
    n->is_ext_static = true;
    int axis = n->parent->axis;
    float position = n->parent->position;

    // check parent's external colliders
    if (!(n->is_subtree_static() && n->parent->is_ext_static)) {
      for (int i = n->parent->ext_start; i < n->parent->ext_end; ++i) {
        BboxCollider<T>* p = buffer_[i];
        // exclude static-static pair
        if (n->is_subtree_static() && p->is_static) {
          continue;
        }

        if (n->is_left()) {
          if (p->bbox.min(axis) < position) {
            buffer_[n->ext_end] = p;
            n->ext_end++;
            n->is_ext_static = n->is_ext_static && p->is_static;
          }
        } else {
          if (p->bbox.max(axis) > position) {
            buffer_[n->ext_end] = p;
            n->ext_end++;
            n->is_ext_static = n->is_ext_static && p->is_static;
          }
        }
      }
    }

    // check parent node colliders
    if (!(n->is_subtree_static() && n->parent->is_subtree_static())) {
      for (int i = n->parent->proxy_start; i < n->parent->proxy_end; ++i) {
        BboxCollider<T>* p = proxies_[i];
        // exclude static-static pair
        if (n->is_subtree_static() && p->is_static) {
          continue;
        }

        if (n->is_left()) {
          if (p->bbox.min(axis) < position) {
            buffer_[n->ext_end] = p;
            n->ext_end++;
            n->is_ext_static = n->is_ext_static && p->is_static;
          }
        } else {
          if (p->bbox.max(axis) > position) {
            buffer_[n->ext_end] = p;
            n->ext_end++;
            n->is_ext_static = n->is_ext_static && p->is_static;
          }
        }
      }
    }
  }

  void test_node_collision(KDNode* n,
                           CollisionFilterCallback<T> filter_callback,
                           CollisionCache<T>& cache) {
    int proxy_num = n->proxy_num();
    if (proxy_num == 0) {
      return;
    }

    BboxCollider<T>** proxy_start = proxies_.data() + n->proxy_start;

    if (n->is_static()) {
      if (n->ext_num() != 0 && !n->is_ext_static) {
        // node - external test
        BboxCollider<T>** ext_start = buffer_.data() + n->ext_start;
        int ext_num = n->ext_num();
        int axis =
            sap_find_optimal_axis(proxy_start, proxy_num, ext_start, ext_num);
        sap_sort_proxies(proxy_start, proxy_num, axis);
        sap_sort_proxies(ext_start, ext_num, axis);
        sap_test_sorted_group_collision(proxy_start, proxy_num, ext_start,
                                        ext_num, axis, filter_callback, cache);
      }
    } else {
      if (n->ext_num() == 0) {
        // node - node test
        int axis = sap_find_optimal_axis(proxy_start, proxy_num);
        sap_sort_proxies(proxy_start, proxy_num, axis);
        sap_test_sorted_self_collision(proxy_start, proxy_num, axis,
                                       filter_callback, cache);

      } else {
        // node - node + node - external test
        BboxCollider<T>** ext_start = buffer_.data() + n->ext_start;
        int ext_num = n->ext_num();
        int axis =
            sap_find_optimal_axis(proxy_start, proxy_num, ext_start, ext_num);
        sap_sort_proxies(proxy_start, proxy_num, axis);
        sap_sort_proxies(ext_start, ext_num, axis);
        sap_test_sorted_self_collision(proxy_start, proxy_num, axis,
                                       filter_callback, cache);
        sap_test_sorted_group_collision(proxy_start, proxy_num, ext_start,
                                        ext_num, axis, filter_callback, cache);
      }
    }
  }
};

}  // namespace silk
