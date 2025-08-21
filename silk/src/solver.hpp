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
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  int thread_num = 4;
  int r = 30;
  float dt = 1.0f;
  float eps = 1e-6f;

 private:
  int state_num_ = 0;

  Eigen::VectorXf init_state_;
  Eigen::VectorXf curr_state_;
  Eigen::VectorXf state_velocity_;

  Eigen::SparseMatrix<float> mass_;
  Eigen::SparseMatrix<float> H_;  // use row major format for iterative solver
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
  bool step(Registry& registry, CollisionPipeline& collision_pipeline);

 private:
  bool lg_solve(Registry& registry, const Eigen::VectorXf& init_rhs,
                const Eigen::SparseMatrix<float> dH, Eigen::VectorXf& state);
};

}  // namespace silk
