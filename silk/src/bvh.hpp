#pragma once

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdint>
#include <iterator>
#include <optional>
#include <vector>

#include "bbox.hpp"

namespace silk {

template <typename T>
struct BVHNode {
  std::optional<T> data;
  Bbox bbox;
  BVHNode* parent = nullptr;
  BVHNode* left = nullptr;
  BVHNode* right = nullptr;

  bool is_leaf() const {
    // bvh tree is always balanced, testing one child is enough
    return !left;
  }
};

template <typename T>
class BVHTree {
  using NodeIter = typename std::vector<BVHNode<T>*>::iterator;

  BVHNode<T>* true_root_ = nullptr;
  BVHNode<T>* node_cache_ = nullptr;
  uint64_t path_ = 0;
  static constexpr uint32_t PATH_BIT_COUNTER_MAX_ = (1u << 6) - 1u;
  int leaf_num_ = 0;
  int reinsert_lookahead_ = -1;

  static void delete_subtree(BVHNode<T>* root) {
    while (root) {
      delete_subtree(root->left);
      delete_subtree(root->right);
      delete root;
    }
  }

  // extract all leaf nodes and delete internal nodes
  static void extract_subtree_leaves(BVHNode<T>* root,
                                     std::vector<BVHNode<T>*>& leaves) {
    if (root->is_leaf()) {
      leaves.push_back(root);
    } else {
      assert(root->left && root->right &&
             "internal root should always have two children");

      extract_leaves_recursively(root->left);
      extract_leaves_recursively(root->right);
      delete root;
    }
  }

  // insert node under root, return new root
  BVHNode<T>* subtree_insert(BVHNode<T>* root, BVHNode<T>* node) {
    // root is empty
    if (!root) {
      node->parent = nullptr;
      return node;
    }

    // descend to leaf based on proximity heuristic
    BVHNode<T>* current = root;
    while (!current->is_leaf()) {
      assert(current->left && current->right &&
             "internal node should always have two children");

      float a = Bbox::proximity(node, current->left->bbox);
      float b = Bbox::proximity(node, current->right->bbox);
      current = (a < b) ? current->left : current->right;
    }

    assert(!current->left && !current->right &&
           "leaf node should have no children");

    // current is now a leaf node.
    // create internal node and set node and current as its children.
    BVHNode<T>* parent = current->parent;

    // reuse node cache if available
    BVHNode<T>* internal = nullptr;
    if (node_cache_) {
      std::swap(internal, node_cache_);
      *internal = {.data = std::nullopt,
                   .bbox = Bbox::merge(node->bbox, current->bbox),
                   .parent = parent,
                   .left = current,
                   .right = node};
    } else {
      internal = new BVHNode<T>{.data = std::nullopt,
                                .bbox = Bbox::merge(node->bbox, current->bbox),
                                .parent = parent,
                                .left = current,
                                .right = node};
    }

    // if current has parent, it now points to internal
    if (parent) {
      if (parent->left == current) {
        parent->left = internal;
      } else {
        parent->right = internal;
      }
    }

    current->parent = internal;
    node->parent = internal;

    // walk back up the tree, refitting bbox
    while (parent && !parent->bbox.contain(node->bbox)) {
      parent->bbox.merge_inplace();
      parent = parent->parent;
    }

    return (current == root) ? internal : root;
  }

  static BVHNode<T>* bottom_up_build(NodeIter begin, NodeIter end) {
    while (std::distance(begin, end) > 1) {
      // Find the pair of nodes that, when merged, create the smallest bbox
      Bbox min_bbox;
      float min_diag2 = std::numeric_limits<float>::max();
      NodeIter min_it1, min_it2;
      for (NodeIter it1 = begin; it1 != end; ++it1) {
        for (NodeIter it2 = it1 + 1; it2 != end; ++it2) {
          Bbox bbox = Bbox::merge((*it1)->bbox, (*it2)->bbox);
          float diag2 = (bbox.min - bbox.max).squaredNorm();
          if (diag2 < min_diag2) {
            min_bbox = bbox;
            min_diag2 = diag2;
            min_it1 = it1;
            min_it2 = it2;
          }
        }
      }

      // merge 2 nodes
      auto node = new BVHNode<T>{.data = std::nullopt,
                                 .bbox = min_bbox,
                                 .parent = nullptr,
                                 .left = *min_it1,
                                 .right = *min_it2};
      (*min_it1)->parent = node;
      (*min_it2)->parent = node;

      // Replace one of the merged nodes with the new parent and shrink the
      // vector.
      std::swap(*min_it1, *begin);
      *min_it2 = node;
      begin++;
    }

    assert(std::distance(begin, end) == 1);
    return *begin;
  }

  static BVHNode<T>* top_down_build(NodeIter begin, NodeIter end,
                                    int bottom_up_threshold) {
    int num = std::distance(begin, end);
    assert((num > 0));

    // reaching leaf
    if (num == 1) {
      return *begin;
    }

    if (num <= bottom_up_threshold) {
      return bottom_up_build(begin, end);
    }

    Bbox bbox;
    for (NodeIter it = begin; it != end; ++it) {
      bbox.merge_inplace((*it)->bbox);
    }

    // split leaves by bbox center
    Eigen::Vector3f center = bbox.center();
    // split_count[axis][0] = leaves at the left of axis plane
    // split_count[axis][1] = leaves at the right of axis plane
    Eigen::Matrix<int, 3, 2> split_count = Eigen::Matrix<int, 3, 2>::Zero();
    for (NodeIter it = begin; it != end; ++it) {
      Eigen::Vector3f delta = (*it)->bbox.center() - center;
      split_count(0, (delta(0) > 0) ? 1 : 0)++;
      split_count(1, (delta(1) > 0) ? 1 : 0)++;
      split_count(2, (delta(2) > 0) ? 1 : 0)++;
    }

    // choose the axis plane that split the leaves most evenly
    Eigen::Vector3i split_count_diff =
        (split_count.col(0) - split_count.col(1)).cwiseAbs();
    int split_axis;
    int min_diff = split_count_diff.maxCoeff(&split_axis);
    NodeIter right_begin;
    // if all axis planes split nothing, choose a random split at middle
    if (min_diff == 0) {
      right_begin = std::advance(begin, num / 2 + 1);
    }
    // partition by axis plane
    else {
      auto pred = [&center, split_axis](BVHNode<T>* node) -> bool {
        return (node->bbox.center() - center).coeff(split_axis) < 0;
      };
      right_begin = std::partition(begin, end, pred);
      assert((right_begin != begin && right_begin != end));
    }

    auto node = new BVHNode<T>{.data = std::nullopt,
                               .bbox = bbox,
                               .parent = std::nullopt,
                               .left = top_down_build(begin, right_begin),
                               .right = top_down_build(right_begin, end)};
    node->left->parent = node;
    node->right->parent = node;
    return node;
  }

  static BVHNode<T>* rotate(BVHNode<T>* node) {
    assert(node->parent && node->left && node->right);

    BVHNode<T>* parent = node->parent;

    if (node == parent->left) {
      parent->left = node->right;
      node->right = parent;
    } else {
      parent->right = node->left;
      node->left = parent;
    }

    node->parent = parent->parent;
    parent->parent = node;
    return node;
  }

  // optimize memory layout for BFS.
  void optimize_memory_layout() {
    assert(true_root_);
    assert(!true_root_->is_leaf());

    uint32_t path_bit_counter = 0;
    auto descend = [path = path_,
                    &path_bit_counter](BVHNode<T>* node) -> BVHNode<T>* {
      uint32_t choice = (path >> path_bit_counter) & 1u;
      path_bit_counter = (path_bit_counter + 1) & PATH_BIT_COUNTER_MAX_;
      return (choice == 0) ? node->left : node->right;
    };

    // special case, might rotate root
    BVHNode<T>* node = descend(true_root_);
    if (!node->is_leaf() && true_root_ > node) {
      node = rotate(node);
      true_root_ = node;
    }

    if (!node) {
      return;
    }

    while (!node->is_leaf()) {
      if (node->parent > node) {
        rotate(node);
      }
      node = descend(node);
    }
  }

 public:
  BVHTree() = default;
  ~BVHTree() { clear(); }

  void clear() {
    delete_tree(true_root_);
    true_root_ = nullptr;
    if (node_cache_) {
      delete node_cache_;
    }
    leaf_num_ = 0;
  }

  void optimize_top_down(int bottom_up_threshold = 128) {
    assert((bottom_up_threshold > 1));

    if (!true_root_) {
      return;
    }

    // destroy current tree by extracting all leaf nodes
    std::vector<BVHNode<T>*> leaves;
    leaves.reserve(leaf_num_);
    extract_subtree_leaves(true_root_, leaves);

    // rebuild top down
    true_root_ =
        top_down_build(leaves.begin(), leaves.end(), bottom_up_threshold);
    assert(true_root_);
    true_root_->parent = nullptr;
  }

  void optimize_incremental(int passes) {
    assert((passes >= -1));

    if (!true_root_) {
      return;
    }

    if (passes == -1) {
      passes = leaf_num_;
    }

    while (passes > 0) {
      optimize_memory_layout();
      path_++;
      passes--;
    }
  }

  const BVHNode<T>* insert(T data, Bbox bbox) {
    auto node = new BVHNode<T>{.data = std::move(data),
                               .bbox = std::move(bbox),
                               .parent = nullptr,
                               .left = nullptr,
                               .right = nullptr};
    subtree_insert(true_root_, node);
    return node;
  }

  void update(BVHNode<T>* node, Bbox bbox);

  void remove(BVHNode<T>* node);
};

}  // namespace silk
