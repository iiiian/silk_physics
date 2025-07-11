#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstring>
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
  int static_num;         // static node proxy num
  int population;         // subtree proxy num
  int static_population;  // static subtree proxy num
  int delay_offset;       // delayed update to proxy start/end
  int ext_start;          // external collider buffer start
  int ext_end;            // external collider buffer end
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
  using Proxy = const BboxCollider<T>*;

  static constexpr int NODE_PROXY_NUM_THRESHOLD = 512;
  static constexpr int APPROX_PLANE_SAMPLE_NUM = 16;
  static constexpr int APPROX_PLANE_SAMPLE_THRESHOLD = 32;

  int collider_num_;
  const BboxCollider<T>* colliders_;

 public:
  KDTree(const BboxCollider<T>* colliders, int collider_num) {
    assert(colliders);
    assert((collider_num > 0));

    collider_num_ = collider_num;
    proxies_.resize(collider_num);
    buffer_.resize(collider_num);
    for (int i = 0; i < collider_num_; ++i) {
      proxies_[i] = colliders + i;
    }
    root_ = new KDNode{.parent = nullptr,
                       .left = nullptr,
                       .right = nullptr,
                       .bbox = {},
                       .axis = 0,
                       .position = 0,
                       .proxy_start = 0,
                       .proxy_end = collider_num_,
                       .static_num = 0,
                       .population = collider_num_,
                       .static_population = 0,
                       .delay_offset = 0,
                       .ext_start = 0,
                       .ext_end = 0,
                       .is_ext_static = false};
    set_root_bbox();
  }

  ~KDTree() { delete_subtree(root_); }

  void update() {
    set_root_bbox();
    lift_unfit_up();
    optimize_structure();
    set_static_population();
  }

  void test_self_collision(CollisionFilterCallback<T> filter_callback,
                           CollisionCache<T>& cache) {
    assert(stack_.empty());

    root_->ext_start = 0;
    root_->ext_end = 0;
    test_node_collision(root_, filter_callback, cache);

    if (!root_->is_leaf()) {
      stack_.push_back(root_->right);
      stack_.push_back(root_->left);
    }

    while (!stack_.empty()) {
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
        Proxy* start_a = ta.proxies_.data() + na->proxy_start;
        int num_a = na->proxy_num();
        Proxy* start_b = tb.proxies_.data() + nb->proxy_start;
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
  KDNode* root_;
  std::vector<KDNode*> stack_;  // for tree traversal
  std::vector<Proxy> proxies_;  // in-order layout proxy array
  std::vector<Proxy> buffer_;   // for both proxies and external colliders

  void set_root_bbox() {
    root_->bbox = colliders_[0].bbox;
    for (int i = 1; i < collider_num_; ++i) {
      root_->bbox.merge_inplace(colliders_[i].bbox);
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
  // plane axis should has the max variance and the plane position is the
  // mean.
  void find_optimal_plane(KDNode* n) const {
    assert((n->proxy_num() > 0));

    auto [mean, variance] =
        proxy_mean_variance(proxies_.data() + n->proxy_start, n->proxy_num());
    variance.maxCoeff(&n->axis);
    n->position = mean(n->axis);
  }

  // find approximate split plane based on mean and variance of bbox center.
  // sample a few bbox only.
  void find_approx_plane(KDNode* n) {
    static std::random_device rand_device;
    static std::mt19937 rand_generator(rand_device());

    assert((n->proxy_num() > 0));

    if (n->proxy_num() <= APPROX_PLANE_SAMPLE_THRESHOLD) {
      find_optimal_plane(n);
      return;
    }

    int start = n->proxy_start;
    int end = n->proxy_start + APPROX_PLANE_SAMPLE_NUM;
    for (int i = start; i < end; ++i) {
      std::uniform_int_distribution rand_dist(i, n->proxy_end - 1);
      int rand_idx = rand_dist(rand_generator);
      std::swap(proxies_[i], proxies_[rand_idx]);
    }

    auto [mean, variance] =
        proxy_mean_variance(proxies_.data() + start, APPROX_PLANE_SAMPLE_NUM);
    variance.maxCoeff(&n->axis);
    n->position = mean(n->axis);
  }

  int push_unfit_proxy_left(const KDNode* n) {
    auto is_outside = [n](Proxy p) -> bool {
      return !(n->bbox.is_inside(p->bbox));
    };

    Proxy start = proxies_.data() + n->proxy_start;
    Proxy end = proxies_.data() + n->proxy_end;
    Proxy new_start = std::partition(start, end, is_outside);

    return new_start - start;
  }

  int push_unfit_proxy_right(const KDNode* n) {
    auto is_inside = [n](Proxy p) -> bool {
      return n->bbox.is_inside(p->bbox);
    };

    Proxy start = proxies_.data() + n->proxy_start;
    Proxy end = proxies_.data() + n->proxy_end;
    Proxy new_end = std::partition(start, end, is_inside);

    return end - new_end;
  }

  void shift_proxy_left(int proxy_start, int proxy_num, int shift_num) {
    if (proxy_num == 0 || shift_num == 0) {
      return;
    }

    size_t copy_size = proxy_num * sizeof(Proxy);
    size_t shift_size = shift_num * sizeof(Proxy);
    Proxy left = proxies_.data() + proxy_start - shift_num;
    Proxy right = proxies_.data() + proxy_start;
    buffer_.resize(copy_size);

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

    size_t copy_size = proxy_num * sizeof(Proxy);
    size_t shift_size = shift_num * sizeof(Proxy);
    Proxy left = proxies_.data() + proxy_start;
    Proxy right = proxies_.data() + proxy_start + proxy_num;
    buffer_.resize(proxy_num);

    // copy left chunk to temp buffer
    std::memcpy(buffer_.data(), left, copy_size);
    // shift right chunk left
    std::memmove(left, right, shift_size);
    // copy left chunk back
    std::memcpy(left + shift_num, buffer_.data(), copy_size);
  }

  void lift_unfit_up() {
    assert(stack_.empty());

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
          int unfit_num = push_unfit_proxy_right(n);
          n->proxy_end -= unfit_num;
          n->parent->proxy_start -= unfit_num;
          n->population = n->proxy_num();
        } else {
          int unfit_num = push_unfit_proxy_left(n);
          n->proxy_start += unfit_num;
          n->parent->proxy_end += unfit_num;
          n->population = n->proxy_num();
        }
      } else {
        if (n->is_left()) {
          int unfit_num = push_unfit_proxy_right(n);
          shift_proxy_right(n->proxy_end - unfit_num, unfit_num,
                            n->right->population);

          n->proxy_end -= unfit_num;
          n->parent->proxy_start -= unfit_num;
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
          n->right->delay_offset = -unfit_num;
        } else {
          int unfit_num = push_unfit_proxy_left(n);
          shift_proxy_left(n->proxy_start, unfit_num, n->left->population);

          n->proxy_start += unfit_num;
          n->parent->proxy_end += unfit_num;
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

    stack_.clear();
  }

  void filter(KDNode* n) {
    assert((n->left && n->right));

    // partition proxies by plane.
    // strictly left  -> move to left partition
    // on the plane   -> move to middle partition
    // strictly right -> move to right partition
    int left_end = n->proxy_start;
    int middle_end = n->proxy_end;
    int i = n->proxy_start;
    while (i != middle_end) {
      Proxy p = proxies_[i];

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
    }
    n->left->proxy_end += left_num;
    n->left->population += left_num;
    n->left->right->proxy_start += left_num;
    n->left->right->proxy_end += left_num;

    // move right partition down one node level
    int right_num = n->proxy_end - middle_end;
    if (!n->right->is_leaf()) {
      shift_proxy_right(middle_end, right_num, n->right->left->population);
    }
    n->right->proxy_start -= right_num;
    n->right->population += right_num;
    n->right->left->proxy_start -= right_num;
    n->right->left->proxy_end -= right_num;

    // update node
    n->proxy_start += left_num;
    n->proxy_end -= right_num;
  }

  void set_children_bbox(KDNode* n) {
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

    find_optimal_plane(n, n->axis, n->position);

    // create new children
    n->left = new KDNode{.parent = n,
                         .left = nullptr,
                         .right = nullptr,
                         .bbox = {},
                         .axis = 0,
                         .position = 0,
                         .proxy_start = n->proxy_start,
                         .proxy_end = n->proxy_start,
                         .static_num = 0,
                         .population = 0,
                         .static_population = 0,
                         .delay_offset = 0,
                         .ext_start = 0,
                         .ext_end = 0,
                         .is_ext_static = false};
    n->right = new KDNode{.parent = n,
                          .left = nullptr,
                          .right = nullptr,
                          .bbox = {},
                          .axis = 0,
                          .position = 0,
                          .proxy_start = n->proxy_end,
                          .proxy_end = n->proxy_end,
                          .static_num = 0,
                          .population = 0,
                          .static_population = 0,
                          .delay_offset = 0,
                          .ext_start = 0,
                          .ext_end = 0,
                          .is_ext_static = false};

    filter(n);
    set_children_bbox(n);
  }

  // collapse subtree into leaf
  void collapse(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    n->proxy_end += n->right->population;

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
      Proxy p = proxies_[i];
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

    float num = n->proxy_num();
    float t_min = 0.5f * num * (0.5f * num - 1.0f);
    float t_max = 0.5f * num * (num - 1.0f);
    float t = 0.5f * (left_num * left_num + middle_num * middle_num +
                      right_num * right_num - num) +
              middle_num * (left_num + right_num);
    float cost = (t - t_min) / (t_max - t_min);
    float balance = float(std::min(left_num, right_num)) /
                    float((middle_num + std::max(left_num, right_num)));
    return (cost <= balance);
  }

  void lift_subtree(KDNode* n) {
    assert((n->left && n->right));

    n->proxy_start -= n->left->population;
    n->proxy_end += n->right->population;

    size_t init_stack_size = stack_.size();

    // lift left subtree
    stack_.push_back(n->left);
    while (stack_.size() > init_stack_size) {
      KDNode* current = stack_.back();
      stack_.pop_back();

      if (!current->is_leaf() && current->population != 0) {
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

      if (!current->is_leaf() && current->population != 0) {
        stack_.push_back(current->left);
        stack_.push_back(current->right);
      }

      current->proxy_end = n->proxy_end;
      current->proxy_end = n->proxy_end;
      current->population = 0;
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

  KDNode* erase(KDNode* n, bool keep_left) {
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

    main->parent = n->parent;
    main->bbox = n->bbox;
    main->proxy_start = n->proxy_start;
    main->proxy_end = n->proxy_end;
    main->population = n->population;
    if (n->parent) {
      if (n->is_left()) {
        n->parent->left = main;
      } else {
        n->parent->right = main;
      }
    }

    delete_subtree(other);
    delete n;

    return main;
  }

  void optimize_structure() {
    assert(stack_.empty());

    // pre-order traverse the tree
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
        stack_.push_back(n->right);
        stack_.push_back(n->left);
        continue;
      }

      // translated plane is still not optimal, erase the node
      stack_.push_back(erase(n, (left_num > right_num)));
    }
  }

  void set_static_population() {
    assert(stack_.empty());

    // prepare for bottom up traversal
    stack_.push_back(root_);
    for (int i = 0; i < stack_.size(); ++i) {
      KDNode* n = stack_[i];
      if (!n->is_leaf()) {
        stack_.push_back(n->right);
        stack_.push_back(n->left);
      }
    }

    // bottom up update bbox and static statistics
    for (int i = stack_.size() - 1; i >= 0; --i) {
      KDNode* n = stack_.back();

      n->static_num = 0;
      for (int i = n->proxy_start; i < n->proxy_end; ++i) {
        if (proxies_[i]->is_static) {
          ++n->static_num;
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

  void update_ext_collider(KDNode* n) {
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
    for (int i = n->parent->ext_start; i < n->parent->ext_end; ++i) {
      Proxy p = buffer_[i];

      if (n->is_left()) {
        if (p->bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
          n->is_ext_static = n->is_ext_static && p->is_static;
        }
      } else {
        if (p->bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
          n->is_ext_static = n->is_ext_static && p->is_static;
        }
      }
    }

    // check parent node colliders
    for (int i = n->parent->proxy_start; i < n->parent->proxy_end; ++i) {
      Proxy p = proxies_[i];

      if (n->is_left()) {
        if (p->bbox.min(axis) < position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
          n->is_ext_static = n->is_ext_static && p->is_static;
        }
      } else {
        if (p->bbox.max(axis) > position) {
          buffer_[n->ext_end] = p;
          ++n->ext_end;
          n->is_ext_static = n->is_ext_static && p->is_static;
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

    Proxy* proxy_start = proxies_.data() + n->proxy_start;

    if (n->is_static()) {
      if (n->ext_num() != 0 && !n->is_ext_static) {
        // node - external test
        Proxy* ext_start = buffer_.data() + n->ext_start;
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
        Proxy* ext_start = buffer_.data() + n->ext_start;
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
