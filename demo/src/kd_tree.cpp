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
  assert((size != 0));
  assert((axis < 3));

  // choose random samples and shuffle them to the start
  int sample_count = size;
  if (size > MEDIUM_SAMPL_NUM) {
    sample_count = MEDIUM_SAMPL_NUM;
    fisher_yates_shuffle(begin, end, MEDIUM_SAMPL_NUM);
  }

  // sort random samples based on axis
  auto comp = [&V = V_, axis](const int &a, const int &b) -> bool {
    return (V(a, axis) < V(b, axis));
  };
  std::sort(begin, begin + sample_count, comp);

  // put medium at the start
  std::swap(*begin, *(begin + sample_count / 2));

  return *begin;
}

KDNode *KDTree::build_sub_tree(Iter begin, Iter end, int depth,
                               KDNode *parent) {
  stats_.node_num++;

  auto p_node = new KDNode();
  p_node->depth = depth;
  p_node->parent = parent;
  p_node->axis = (parent == nullptr) ? 0 : next_axis(parent->axis);

  // reaching leaf
  if (depth + 1 == MAX_DEPTH || std::distance(begin, end) <= LEAF_VERT_NUM) {
    p_node->leaf_verts_idx = std::vector<int>(begin, end);

    stats_.current_max_depth = std::max(stats_.current_max_depth, depth);
    stats_.leaf_node_num++;
    stats_.leaves_depth_counter[depth]++;
    if (std::distance(begin, end) > LEAF_VERT_NUM) {
      stats_.depth_terminate_leaf_num++;
    }
    return p_node;
  }

  // find medium
  p_node->node_vert_idx = heuristic_median(begin, end, p_node->axis);
  p_node->node_vert = V_.row(p_node->node_vert_idx);

  // partition the children into left and right
  Iter child_begin = std::next(begin);
  Iter child_end = end;
  auto pred = [&V = V_, p_node](const int &idx) -> bool {
    return (V(idx, p_node->axis) < p_node->node_vert(p_node->axis));
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
  if (balance_ratio < UNBALANCE_THRESHOLD) {
    stats_.unbalance_node_num++;
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

KDTree::KDTree(Verts V) {
  assert((V.rows() != 0));

  V_ = std::move(V);
  stats_.leaves_depth_counter.resize(MAX_DEPTH, 0);

  std::vector<int> verts_idx(V_.rows());
  for (int i = 0; i < V_.rows(); ++i) {
    verts_idx[i] = i;
  }

  p_root_ = build_sub_tree(verts_idx.begin(), verts_idx.end(), 0, nullptr);
}

KDTree::~KDTree() { delete_tree(p_root_); }

void KDTree::update_closest(const Eigen::RowVector3f &point,
                            const KDNode *p_node, float &min_dist_square,
                            int &target_idx) const {
  if (p_node == nullptr) {
    return;
  }

  // reaching leaf
  if (p_node->left == nullptr && p_node->right == nullptr) {
    for (auto idx : p_node->leaf_verts_idx) {
      float dist_square = (point - V_.row(idx)).squaredNorm();
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
      float dist_square = (point - V_.row(idx)).squaredNorm();
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

std::vector<int> KDTree::find_neighbors(const Eigen::RowVector3f &point,
                                        float radius) const {
  std::vector<int> result;
  update_neighbors(point, radius * radius, p_root_, result);
  return result;
}

int KDTree::find_closest(const Eigen::RowVector3f &point) const {
  float min_dist_square = std::numeric_limits<float>::max();
  int target_idx = 0;
  update_closest(point, p_root_, min_dist_square, target_idx);

  return target_idx;
}

KDTreeStatistic KDTree::get_statistic() const { return stats_; }
