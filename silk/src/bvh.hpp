#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <optional>
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
  int axis;
  float position;

  int proxy_start;
  int proxy_end;
  int colli_start;
  int colli_end;

  int static_num;
  int population;
  int static_population;
  int delay_offset;

  bool is_leaf() const {
    // bvh tree is always balanced, testing one child is enough
    return !left;
  }

  bool is_left() const { return (parent && this == parent->left); }

  int proxy_num() const {
    assert((proxy_end >= proxy_start));
    return proxy_end - proxy_start;
  }
};

template <typename T>
class KDTree {
  using Collider = BboxCollider<T>;

  KDNode* true_root_ = nullptr;
  std::vector<KDNode*> stack_;
  std::vector<Collider*> proxies_;
  std::vector<Collider*> buffer_;
  std::vector<Collider> colliders_;

  int collider_num_;
  int static_collider_num_;
  float bbox_margin = 0;
  float incremental_update_threshold = 0;
  int node_collider_num_threshold = 0;
  std::function<bool(const T&, const T&)> is_potential_collision;
  std::vector<std::pair<T*, T*>> cache_;

  void delete_subtree(KDNode* root) {
    if (!root) {
      return;
    }

    size_t init_stack_size = stack_.size();
    stack_.push_back(root);
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

  void clear() {
    delete_subtree(true_root_);
    true_root_ = nullptr;
    stack_.clear();
    proxies_.clear();
    buffer_.clear();
    colliders_.clear();
    static_collider_num_ = 0;
  }

  void init(const std::vector<T>& data, const std::vector<Bbox>& bboxes) {
    clear();

    collider_num_ = data.size();
    true_root_ = new KDNode{.parent = nullptr,
                            .left = nullptr,
                            .right = nullptr,
                            .bbox = Bbox::make_inf_bbox(),
                            .axis = 0,
                            .position = 0,
                            .proxy_start = 0,
                            .proxy_end = collider_num_,
                            .colli_start = 0,
                            .colli_end = 0,
                            .static_num = 0,
                            .population = collider_num_,
                            .static_population = 0,
                            .delay_offset = 0};
    for (int i = 0; i < collider_num_; ++i) {
      colliders_.emplace_back({.node = nullptr,
                               .bbox = Bbox::extend(bboxes[i], bbox_margin),
                               .data = data[i]});
    }
    for (auto& o : colliders_) {
      o.bbox.extend_inplace(bbox_margin);
    }
    buffer_.resize(2 * collider_num_);
  }

  void update(const std::vector<Bbox>& bboxes) {
    assert((bboxes.size() == collider_num_));

    // update objects' bbox
    static_collider_num_ = 0;
    for (int i = 0; i < bboxes.size(); ++i) {
      auto& o = colliders_[i];
      if (o.bbox.is_inside(bboxes[i])) {
        o.is_static = true;
        static_collider_num_++;
      } else {
        o.bbox = Bbox::extend(bboxes[i], bbox_margin);
        o.is_static = false;
      }
    }

    is_incremental_update_ =
        (static_collider_num_ > incremental_update_threshold * collider_num_);

    if (static_collider_num_ != collider_num_) {
      refit();
      optimize();
    }

    update_static_population();
  }

  // void query_collision() {
  //   if (static_obj_num_ == obj_num_) {
  //     return;
  //   }
  //
  //   true_root_->colli_start = 0;
  //   true_root_->colli_end = 0;
  //   assert((true_root_->proxy_num() > 0));
  //   float position;
  //   find_optimal_split_plane(true_root_, true_root_->sap_axis, position);
  //
  //   sort_node_objects(true_root_);
  //   sap_node_node(true_root_);
  // }

  void update_static_population() {
    // BFS stack fill
    stack_.push_back(true_root_);
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

  void shift_proxy_left_to_right(int proxy_start, int proxy_num,
                                 int shift_num) {
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

  void shift_proxy_right_to_left(int proxy_start, int proxy_num,
                                 int shift_num) {
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

  void refit() {
    assert(stack_.empty());

    // BFS stack fill
    stack_.push_back(true_root_);
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
          shift_proxy_left_to_right(n->proxy_end, unfit_num,
                                    n->right->population);
          n->proxy_end -= unfit_num;
          n->parent->proxy_start -= unfit_num;
          n->right->delay_offset = -unfit_num;
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
        } else {
          // right internal
          int unfit_num = push_unfit_left(n);
          shift_proxy_left_to_right(n->proxy_start, unfit_num,
                                    n->left->population);
          n->proxy_start += unfit_num;
          n->parent->proxy_end += unfit_num;
          n->left->delay_offset = +unfit_num;
          n->population =
              n->proxy_num() + n->left->population + n->right->population;
        }
      }
    }

    if (true_root_->is_leaf()) {
      true_root_->population = proxies_.size();
    } else {
      true_root_->population = true_root_->proxy_num() +
                               true_root_->left->population +
                               true_root_->right->population;
    }

    stack_.clear();
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
        shift_proxy_right_to_left(n->proxy_start, left_num,
                                  n->left->right->population);
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
        shift_proxy_left_to_right(right_start, right_num,
                                  n->right->left->population);
        n->right->left->proxy_start -= right_num;
        n->right->left->proxy_end -= right_num;
      }
    }
  }

  void find_optimal_split_plane(KDNode* n, int& axis, float& position) const {
    // find optimal split plane based on mean and variance of bbox center.
    // plane axis should has the max variance and the plane position is the
    // mean.
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (int i = n->proxy_start; i < n->proxy_end; ++i) {
      Eigen::Vector3f center = proxies_[i].bbox.center();
      mean += center;
      variance += center.cwiseAbs2();
    }
    int num = n->proxy_end - n->proxy_start;
    mean /= num;
    variance = variance / num - mean.cwiseAbs2();
    variance.maxCoeff(&axis);
    position = mean(axis);
  }

  // find optimal split plane then split the leaf
  void split_leaf_node(KDNode* n) {
    assert(!(n->left || n->right));
    assert((n->proxy_end - n->proxy_start > 0));

    find_optimal_split_plane(n, n->axis, n->position);

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
                         .colli_start = 0,
                         .colli_end = 0,
                         .static_num = 0,
                         .population = 0,
                         .static_population = 0,
                         .delay_offset = 0};
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
                          .colli_start = 0,
                          .colli_end = 0,
                          .static_num = 0,
                          .population = 0,
                          .static_population = 0,
                          .delay_offset = 0};

    split_internal_node(n);
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

  bool evaluate(float lp, float mp, float rp) const {
    float s = lp + mp + rp;
    float t_min = 0.5f * s * (0.5f * s - 1.0f);
    float t_max = 0.5f * s * (s - 1.0f);
    float t = 0.5 * (lp * lp + mp * mp + rp * rp - s) + mp * (lp + rp);
    float cost = (t - t_min) / (t_max - t_min);
    float balance = std::min(lp, rp) / (mp + std::max(lp, rp));
    return (cost <= balance);
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
    stack_.push_back(true_root_);
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
        if (n->proxy_num() > node_collider_num_threshold) {
          split_leaf_node(n);
          stack_.push_back(n->right);
          stack_.push_back(n->left);
        }
        continue;
      }

      // if population of internal nodes is too small, collapse into leaf node
      if (n->population < node_collider_num_threshold) {
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

  void sort_node_objects(KDNode* n, int axis) {
    auto comp = [axis](KDNode* a, KDNode* b) -> bool {
      return (a->bbox.min(axis) < b->bbox.min(axis));
    };
    auto start_iter = std::advance(proxies_.begin(), n->proxy_start);
    auto end_iter = std::advance(proxies_.begin(), n->proxy_end);
    std::sort(start_iter, end_iter, comp);
  }

  void sap_node_node(KDNode* n) {
    if (n->static_num == n->proxy_num()) {
      return;
    }

    for (int i = n->proxy_start; i < n->proxy_end - 1; ++i) {
      sap_array(proxies_[i], i + 1, n->proxy_end, proxies_, 0);
    }
  }

  void sap_node_external(KDNode* n);
};

}  // namespace silk
