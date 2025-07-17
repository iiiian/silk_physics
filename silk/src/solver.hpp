#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include <vector>

namespace silk {

class SolverConstrain {
 public:
  virtual ~SolverConstrain() = default;

  virtual void project(const Eigen::VectorXf& position,
                       Eigen::VectorXf& out) const = 0;
};

class PositionConstrain : public SolverConstrain {
  Eigen::VectorXi vert_indexes_;
  int offset_;
  float weight_;

 public:
  PositionConstrain(Eigen::VectorXi vert_indexes, int offset, float weight);

  // impl solver constrain interface
  void project(const Eigen::VectorXf& position,
               Eigen::VectorXf& out) const override;
};

struct SolverInitData {
  std::vector<Eigen::Triplet<float>> mass;
  std::vector<Eigen::Triplet<float>> weighted_AA;
  std::vector<std::unique_ptr<SolverConstrain>> constrains;
};

}  // namespace silk
