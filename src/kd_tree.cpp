#include "kd_tree.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <random>

namespace eg = Eigen;

eg::Index KDTree::next_axis(eg::Index idx) { return (idx == 2) ? 0 : idx + 1; }

void KDTree::fisher_yates_shuffle(std::span<eg::Index> verts_idx,
                                  size_t n) const {
  assert((verts_idx.size() != 0));
  assert((n <= verts_idx.size()));
  assert((n != 0));

  static std::random_device rand_device;
  static std::mt19937 rand_generator(rand_device());

  for (size_t i = 0; i < n - 1; ++i) {
    std::uniform_int_distribution<size_t> rand_dist(i, verts_idx.size() - 1);
    size_t rand_idx = rand_dist(rand_generator);

    std::swap(verts_idx[i], verts_idx[rand_idx]);
  }
}

eg::Index KDTree::heuristic_median(std::span<eg::Index> verts_idx,
                                   eg::Index axis) const {
  assert((median_sample_num != 0));
  assert((verts_idx.size() != 0));
  assert((axis < 3));

  // choose random samples and shuffle them to the start
  size_t sample_count = verts_idx.size();
  if (verts_idx.size() > median_sample_num) {
    sample_count = median_sample_num;
    fisher_yates_shuffle(verts_idx, median_sample_num);
  }

  // sort random samples based on axis
  auto comp = [this, &axis](const eg::Index &a, const eg::Index &b) -> bool {
    const eg::MatrixX3d &verts = *this->_p_verts;
    return (verts(a, axis) < verts(b, axis));
  };
  std::ranges::sort(verts_idx.first(sample_count), comp);

  // put medium at the start
  std::swap(verts_idx[0], verts_idx[sample_count / 2]);

  return verts_idx[0];
}

KDNode *KDTree::build_sub_tree(std::span<eg::Index> verts_idx, size_t depth,
                               KDNode *parent) {
  _stats.node_num++;

  auto p_node = new KDNode();
  p_node->depth = depth;
  p_node->parent = parent;
  p_node->axis = (parent == nullptr) ? 0 : next_axis(parent->axis);

  // reaching leaf
  if (depth + 1 == max_depth || verts_idx.size() <= leaf_vert_num) {
    p_node->leaf_verts_idx =
        std::vector<eg::Index>(verts_idx.begin(), verts_idx.end());

    _stats.current_max_depth = std::max(_stats.current_max_depth, depth);
    _stats.leaf_node_num++;
    _stats.leaves_depth_counter[depth]++;
    if (verts_idx.size() > leaf_vert_num) {
      _stats.depth_terminate_leaf_num++;
    }
    return p_node;
  }

  // find medium
  p_node->node_vert_idx = heuristic_median(verts_idx, p_node->axis);
  p_node->node_vert = _p_verts->row(p_node->node_vert_idx);

  // partition the children into left and right
  auto child_verts_idx = verts_idx.last(verts_idx.size() - 1);
  auto pred = [this, p_node](const eg::Index &idx) -> bool {
    return (this->_p_verts->coeff(idx, p_node->axis) <
            p_node->node_vert(p_node->axis));
  };
  auto right_begin_it =
      std::partition(child_verts_idx.begin(), child_verts_idx.end(), pred);

  // create left child
  auto left_span = std::span(child_verts_idx.begin(), right_begin_it);
  p_node->left = build_sub_tree(left_span, depth + 1, p_node);

  // create right child
  auto right_span = std::span(right_begin_it, child_verts_idx.end());
  p_node->right = build_sub_tree(right_span, depth + 1, p_node);

  double balance_ratio =
      static_cast<double>(std::min(left_span.size(), right_span.size())) /
      (verts_idx.size() - 1);
  if (balance_ratio < _unbalance_node_threshold) {
    _stats.unbalance_node_num++;
  }

  return p_node;
}

void KDTree::delete_tree(KDNode *p_node) {
  if (p_node == nullptr) {
    return;
  }

  delete_tree(p_node->left);
  delete_tree(p_node->right);
  delete p_node;
}

KDTree::~KDTree() { delete_tree(_p_root); }

void KDTree::update_closest(const eg::RowVector3d &point, const KDNode *p_node,
                            double &min_dist_square,
                            eg::Index &target_idx) const {
  if (p_node == nullptr) {
    return;
  }

  // reaching leaf
  if (p_node->left == nullptr && p_node->right == nullptr) {
    for (auto idx : p_node->leaf_verts_idx) {
      double dist_square = (point - _p_verts->row(idx)).squaredNorm();
      if (dist_square < min_dist_square) {
        min_dist_square = dist_square;
        target_idx = idx;
      }
    }
    return;
  }

  double delta = point(p_node->axis) - p_node->node_vert(p_node->axis);

  // search left
  if (delta < 0 || delta * delta < min_dist_square) {
    update_closest(point, p_node->left, min_dist_square, target_idx);
  }
  // search right
  if (delta > 0 || delta * delta < min_dist_square) {
    update_closest(point, p_node->right, min_dist_square, target_idx);
  }

  // check the distance to the node vertex itself
  double dist_sqaure = (point - p_node->node_vert).squaredNorm();
  if (dist_sqaure < min_dist_square) {
    min_dist_square = dist_sqaure;
    target_idx = p_node->node_vert_idx;
  }
}

void KDTree::update_neighbors(const eg::RowVector3d &point,
                              double radius_square, const KDNode *p_node,
                              std::vector<eg::Index> &result) const {
  if (p_node == nullptr) {
    return;
  }

  // reaching leaf
  if (p_node->left == nullptr && p_node->right == nullptr) {
    for (auto idx : p_node->leaf_verts_idx) {
      double dist_square = (point - _p_verts->row(idx)).squaredNorm();
      if (dist_square < radius_square) {
        result.push_back(idx);
      }
    }
    return;
  }

  double delta = point(p_node->axis) - p_node->node_vert(p_node->axis);
  // search left
  if (delta < 0 || delta * delta < radius_square) {
    update_neighbors(point, radius_square, p_node->left, result);
  }
  // search right
  if (delta >= 0 || delta * delta < radius_square) {
    update_neighbors(point, radius_square, p_node->right, result);
  }
  // check node vertex itself
  double dist_square = (point - p_node->node_vert).squaredNorm();
  if (dist_square < radius_square) {
    result.push_back(p_node->node_vert_idx);
  }
}

void KDTree::init(const eg::MatrixX3d *p_verts) {
  assert((median_sample_num != 0));
  assert((max_depth != 0));
  assert((leaf_vert_num != 0));
  assert((p_verts->rows() != 0));

  clear();
  _p_verts = p_verts;
  _stats.leaves_depth_counter.resize(max_depth, 0);

  std::vector<eg::Index> verts_idx(p_verts->rows());
  for (eg::Index i = 0; i < p_verts->rows(); ++i) {
    verts_idx[i] = i;
  }

  _p_root = build_sub_tree(verts_idx, 0, nullptr);
}

void KDTree::clear() {
  delete_tree(_p_root);
  _p_root = nullptr;
  _p_verts = nullptr;
  _stats = {};
}

std::vector<eg::Index> KDTree::find_neighbors(const eg::RowVector3d &point,
                                              double radius) const {
  assert((_p_root != nullptr));

  std::vector<eg::Index> result;
  update_neighbors(point, radius * radius, _p_root, result);
  return result;
}

eg::Index KDTree::find_closest(const eg::RowVector3d &point) const {
  assert((_p_root != nullptr));

  double min_dist_square = std::numeric_limits<double>::max();
  eg::Index target_idx = 0;
  update_closest(point, _p_root, min_dist_square, target_idx);

  return target_idx;
}

KDTreeStatistic KDTree::get_statistic() const { return _stats; }

const KDNode *KDTree::get_root() const { return _p_root; }
