#pragma once

#include <cassert>
#include <cstdint>
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
  using Node = BVHNode<T>;

  BVHNode<T>* true_root_ = nullptr;
  BVHNode<T>* node_cache_ = nullptr;
  uint64_t path_ = 0;
  uint32_t path_bit_counter_ = 0;
  static constexpr uint32_t PATH_BIT_COUNTER_MAX_ = (1u << 6) - 1u;
  int leaf_num_ = 0;
  int reinsert_lookahead_ = -1;

  static void delete_node_recursively(BVHNode<T>* root) {
    while (root) {
      delete_node_recursively(root->left);
      delete_node_recursively(root->right);
      delete root;
    }
  }

  // extract all leaf nodes and delete internal nodes
  static void extract_leaves(BVHNode<T>* root,
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

  // insert node under root, update and re-link to root if necessary
  void insert_from_root(BVHNode<T>*& root, BVHNode<T>* node) {
    // root is empty
    if (!root) {
      node->parent = nullptr;
      root = node;
      return;
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

    // if current is root, internal becomes the new root
    if (current == root) {
      root = internal;
    }

    current->parent = internal;
    node->parent = internal;

    // walk back up the tree, refitting bbox
    while (parent && !parent->bbox.contain(node->bbox)) {
      parent->bbox.merge_inplace();
      parent = parent->parent;
    }
  }

  static BVHNode<T>* top_down_build(int start, int end,
                                    std::vector<BVHNode<T>*>& leaves) {
    if (start == end) {
      return nullptr;
    }

    Bbox temp;
    for (int i = start; i < end; ++i) {
      temp.merge_inplace(leaves[i]->bbox);
    }
    Eigen::Vector3f center = temp.center();
    Eigen::Matrix<int, 3, 2> spilt_count = Eigen::Matrix<int, 3, 2>::Zero();
    for (int i = start; i < end; ++i) {
      Eigen::Vector3f delta = leaves[i]->bbox.center() - center;
      spilt_count(0, (delta(0) > 0) ? 1 : 0)++;
      spilt_count(1, (delta(1) > 0) ? 1 : 0)++;
      spilt_count(2, (delta(2) > 0) ? 1 : 0)++;
    }

    Eigen::Vector3i count_diff =
        (spilt_count.col(0) - spilt_count.col(1)).cwiseAbs();
    int axis;
    count_diff.maxCoeff(&axis);

    int right_start = start;
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

  void optimize_top_down(int buttom_up_threshold = 128) {
    if (!true_root_) {
      return;
    }

    // destroy current tree by extracting all leaf nodes
    std::vector<BVHNode<T>*> leaves(leaf_num_);
    extract_leaves(true_root_, leaves);
    true_root_ = nullptr;
  }

  void optimize_incremental(int passes);

  const BVHNode<T>* insert(T data, Bbox bbox) {
    auto node = new BVHNode<T>{.data = std::move(data),
                               .bbox = std::move(bbox),
                               .parent = nullptr,
                               .left = nullptr,
                               .right = nullptr};
    insert_from_root(true_root_, node);
    return node;
  }

  void update(BVHNode<T>* node, Bbox bbox);

  void remove(BVHNode<T>* node);
};

}  // namespace silk
