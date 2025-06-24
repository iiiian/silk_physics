#pragma once

#include <Eigen/Core>
#include <vector>

struct KDNode {
  KDNode *parent = nullptr;
  KDNode *left = nullptr;
  KDNode *right = nullptr;

  Eigen::Index axis = 0;
  Eigen::Index node_vert_idx = 0;
  Eigen::RowVector3f node_vert = {0, 0, 0};
  size_t depth = 0;
  // for leaf node only
  std::vector<Eigen::Index> leaf_verts_idx;
};

struct KDTreeStatistic {
  size_t current_max_depth = 0;
  std::vector<size_t> leaves_depth_counter;
  size_t node_num = 0;
  size_t leaf_node_num = 0;
  size_t unbalance_node_num = 0;
  size_t depth_terminate_leaf_num = 0;
};

class KDTree {
  using Iter = std::vector<Eigen::Index>::iterator;

  KDNode *_p_root = nullptr;
  const Eigen::MatrixX3f *_p_verts = nullptr;
  KDTreeStatistic _stats;

  Eigen::Index next_axis(Eigen::Index idx);
  // choose n element randomly and place them at the begining of the span
  void fisher_yates_shuffle(Iter begin, Iter end, size_t n) const;
  // select medium based on random samples and shuffle the span,
  // the medium will always be at the begining
  Eigen::Index heuristic_median(Iter begin, Iter end, Eigen::Index axis) const;

  KDNode *build_sub_tree(Iter begin, Iter end, size_t depth, KDNode *parent);

  void delete_tree(KDNode *p_node);
  void update_closest(const Eigen::RowVector3f &point, const KDNode *p_node,
                      float &min_dist_square, Eigen::Index &target_idx) const;

  void update_neighbors(const Eigen::RowVector3f &point, float radius,
                        const KDNode *p_node,
                        std::vector<Eigen::Index> &result) const;

 public:
  // A node is deemed unbalanced if
  // (vertex num) / (total vertex num) < (threshold)
  const float _unbalance_node_threshold = 0.3;

  size_t median_sample_num = 11;
  size_t max_depth = 15;
  size_t leaf_vert_num = 5;

  KDTree() = default;
  ~KDTree();
  KDTree(KDTree &) = delete;
  KDTree &operator=(KDTree &) = delete;

  void init(const Eigen::MatrixX3f *p_verts);
  void clear();
  std::vector<Eigen::Index> find_neighbors(const Eigen::RowVector3f &point,
                                           float radius) const;
  Eigen::Index find_closest(const Eigen::RowVector3f &point) const;

  KDTreeStatistic get_statistic() const;
  const KDNode *get_root() const;
};
