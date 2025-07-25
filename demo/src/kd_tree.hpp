#pragma once

#include <Eigen/Core>
#include <vector>

struct KDNode {
  KDNode *parent = nullptr;
  KDNode *left = nullptr;
  KDNode *right = nullptr;

  int axis = 0;
  int node_vert_idx = 0;
  Eigen::RowVector3f node_vert = {0, 0, 0};
  int depth = 0;
  // for leaf node only
  std::vector<int> leaf_verts_idx;
};

struct KDTreeStatistic {
  int current_max_depth = 0;
  std::vector<int> leaves_depth_counter;
  int node_num = 0;
  int leaf_node_num = 0;
  int unbalance_node_num = 0;
  int depth_terminate_leaf_num = 0;
};

class KDTree {
  using Iter = std::vector<int>::iterator;

  KDNode *_p_root = nullptr;
  const Eigen::MatrixX3f *_p_verts = nullptr;
  KDTreeStatistic _stats;

  int next_axis(int idx);
  // choose n element randomly and place them at the begining of the span
  void fisher_yates_shuffle(Iter begin, Iter end, int n) const;
  // select medium based on random samples and shuffle the span,
  // the medium will always be at the begining
  int heuristic_median(Iter begin, Iter end, int axis) const;

  KDNode *build_sub_tree(Iter begin, Iter end, int depth, KDNode *parent);

  void delete_tree(KDNode *p_node);
  void update_closest(const Eigen::RowVector3f &point, const KDNode *p_node,
                      float &min_dist_square, int &target_idx) const;

  void update_neighbors(const Eigen::RowVector3f &point, float radius,
                        const KDNode *p_node, std::vector<int> &result) const;

 public:
  // A node is deemed unbalanced if
  // (vertex num) / (total vertex num) < (threshold)
  const float _unbalance_node_threshold = 0.3;

  int median_sample_num = 11;
  int max_depth = 15;
  int leaf_vert_num = 5;

  KDTree() = default;
  ~KDTree();
  KDTree(KDTree &) = delete;
  KDTree &operator=(KDTree &) = delete;

  void init(const Eigen::MatrixX3f *p_verts);
  void clear();
  std::vector<int> find_neighbors(const Eigen::RowVector3f &point,
                                  float radius) const;
  int find_closest(const Eigen::RowVector3f &point) const;

  KDTreeStatistic get_statistic() const;
  const KDNode *get_root() const;
};
