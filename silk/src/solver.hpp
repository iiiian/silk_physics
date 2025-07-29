#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

#include "collision.hpp"
#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "solver_constrain.hpp"

namespace silk {

class Solver {
 public:
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_iteration = 10;
  int thread_num = 4;
  int r = 30;
  float dt = 1.0f;
  float ccd_walkback = 0.8f;

 private:
  int state_num_ = 0;

  Eigen::VectorXf init_state_;
  Eigen::VectorXf curr_state_;
  Eigen::VectorXf prev_state_;

  Eigen::SparseMatrix<float> mass_;
  Eigen::SparseMatrix<float> H_;
  Eigen::MatrixXf UHU_;
  Eigen::MatrixXf U_;
  Eigen::VectorXf HX_;

  std::vector<ISolverConstrain*> constrains_;
  std::vector<Collision> collisions_;

 public:
  const Eigen::VectorXf& get_solver_state() const;
  void clear();
  void reset();
  bool init(Registry& registry);
  bool step(Registry& registry, const CollisionPipeline& collision_pipeline);

 private:
  bool lg_solve(Registry& registry, const Eigen::VectorXf& init_rhs,
                Eigen::VectorXf& state);
};

}  // namespace silk
