#include "kd_tree.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <random>

int KDTree::next_axis(int idx) { return (idx == 2) ? 0 : idx + 1; }

void KDTree::fisher_yates_shuffle(Iter begin, Iter end, int n) const {
  int size = std::distance(begin, end);
  assert((size != 0));
  assert((n <= size));
  assert((n != 0));

  static std::random_device rand_device;
  static std::mt19937 rand_generator(rand_device());

  for (int i = 0; i < n - 1; ++i) {
    std::uniform_int_distribution<int> rand_dist(i, size - 1);
    int rand_idx = rand_dist(rand_generator);

    std::swap(*(begin + i), *(begin + rand_idx));
  }
}

int KDTree::heuristic_median(Iter begin, Iter end, int axis) const {
  int size = std::distance(begin, end);
  assert((median_sample_num != 0));
  assert((size != 0));
  assert((axis < 3));

  // choose random samples and shuffle them to the start
  int sample_count = size;
  if (size > median_sample_num) {
    sample_count = median_sample_num;
    fisher_yates_shuffle(begin, end, median_sample_num);
  }

  // sort random samples based on axis
  auto comp = [this, &axis](const int &a, const int &b) -> bool {
    const Eigen::MatrixX3f &verts = *this->_p_verts;
    return (verts(a, axis) < verts(b, axis));
  };
  std::sort(begin, begin + sample_count, comp);

  // put medium at the start
  std::swap(*begin, *(begin + sample_count / 2));

  return *begin;
}

KDNode *KDTree::build_sub_tree(Iter begin, Iter end, int depth,
                               KDNode *parent) {
  _stats.node_num++;

  auto p_node = new KDNode();
  p_node->depth = depth;
  p_node->parent = parent;
  p_node->axis = (parent == nullptr) ? 0 : next_axis(parent->axis);

  // reaching leaf
  if (depth + 1 == max_depth || std::distance(begin, end) <= leaf_vert_num) {
    p_node->leaf_verts_idx = std::vector<int>(begin, end);

    _stats.current_max_depth = std::max(_stats.current_max_depth, depth);
    _stats.leaf_node_num++;
    _stats.leaves_depth_counter[depth]++;
    if (std::distance(begin, end) > leaf_vert_num) {
      _stats.depth_terminate_leaf_num++;
    }
    return p_node;
  }

  // find medium
  p_node->node_vert_idx = heuristic_median(begin, end, p_node->axis);
  p_node->node_vert = _p_verts->row(p_node->node_vert_idx);

  // partition the children into left and right
  Iter child_begin = std::next(begin);
  Iter child_end = end;
  auto pred = [this, p_node](const int &idx) -> bool {
    return (this->_p_verts->coeff(idx, p_node->axis) <
            p_node->node_vert(p_node->axis));
  };
  auto right_begin = std::partition(child_begin, child_end, pred);

  // create left child
  p_node->left = build_sub_tree(child_begin, right_begin, depth + 1, p_node);

  // create right child
  p_node->right = build_sub_tree(right_begin, child_end, depth + 1, p_node);

  float left_size = std::distance(child_begin, right_begin);
  float right_size = std::distance(right_begin, child_end);
  float total_size = std::distance(begin, end);
  float balance_ratio = std::min(left_size, right_size) / (total_size - 1);
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

void KDTree::update_closest(const Eigen::RowVector3f &point,
                            const KDNode *p_node, float &min_dist_square,
                            int &target_idx) const {
  if (p_node == nullptr) {
    return;
  }

  // reaching leaf
  if (p_node->left == nullptr && p_node->right == nullptr) {
    for (auto idx : p_node->leaf_verts_idx) {
      float dist_square = (point - _p_verts->row(idx)).squaredNorm();
      if (dist_square < min_dist_square) {
        min_dist_square = dist_square;
        target_idx = idx;
      }
    }
    return;
  }

  float delta = point(p_node->axis) - p_node->node_vert(p_node->axis);

  // search left
  if (delta < 0 || delta * delta < min_dist_square) {
    update_closest(point, p_node->left, min_dist_square, target_idx);
  }
  // search right
  if (delta > 0 || delta * delta < min_dist_square) {
    update_closest(point, p_node->right, min_dist_square, target_idx);
  }

  // check the distance to the node vertex itself
  float dist_sqaure = (point - p_node->node_vert).squaredNorm();
  if (dist_sqaure < min_dist_square) {
    min_dist_square = dist_sqaure;
    target_idx = p_node->node_vert_idx;
  }
}

void KDTree::update_neighbors(const Eigen::RowVector3f &point,
                              float radius_square, const KDNode *p_node,
                              std::vector<int> &result) const {
  if (p_node == nullptr) {
    return;
  }

  // reaching leaf
  if (p_node->left == nullptr && p_node->right == nullptr) {
    for (auto idx : p_node->leaf_verts_idx) {
      float dist_square = (point - _p_verts->row(idx)).squaredNorm();
      if (dist_square < radius_square) {
        result.push_back(idx);
      }
    }
    return;
  }

  float delta = point(p_node->axis) - p_node->node_vert(p_node->axis);
  // search left
  if (delta < 0 || delta * delta < radius_square) {
    update_neighbors(point, radius_square, p_node->left, result);
  }
  // search right
  if (delta >= 0 || delta * delta < radius_square) {
    update_neighbors(point, radius_square, p_node->right, result);
  }
  // check node vertex itself
  float dist_square = (point - p_node->node_vert).squaredNorm();
  if (dist_square < radius_square) {
    result.push_back(p_node->node_vert_idx);
  }
}

void KDTree::init(const Eigen::MatrixX3f *p_verts) {
  assert((median_sample_num != 0));
  assert((max_depth != 0));
  assert((leaf_vert_num != 0));
  assert((p_verts->rows() != 0));

  clear();
  _p_verts = p_verts;
  _stats.leaves_depth_counter.resize(max_depth, 0);

  std::vector<int> verts_idx(p_verts->rows());
  for (int i = 0; i < p_verts->rows(); ++i) {
    verts_idx[i] = i;
  }

  _p_root = build_sub_tree(verts_idx.begin(), verts_idx.end(), 0, nullptr);
}

void KDTree::clear() {
  delete_tree(_p_root);
  _p_root = nullptr;
  _p_verts = nullptr;
  _stats = {};
}

std::vector<int> KDTree::find_neighbors(const Eigen::RowVector3f &point,
                                        float radius) const {
  assert((_p_root != nullptr));

  std::vector<int> result;
  update_neighbors(point, radius * radius, _p_root, result);
  return result;
}

int KDTree::find_closest(const Eigen::RowVector3f &point) const {
  assert((_p_root != nullptr));

  float min_dist_square = std::numeric_limits<float>::max();
  int target_idx = 0;
  update_closest(point, _p_root, min_dist_square, target_idx);

  return target_idx;
}

KDTreeStatistic KDTree::get_statistic() const { return _stats; }

const KDNode *KDTree::get_root() const { return _p_root; }
