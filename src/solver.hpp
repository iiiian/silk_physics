#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <unordered_set>
#include <vector>

// #include "collision_detector.hpp"
#include "common_types.hpp"

class ClothSolver {
 private:
  RMatrixX3f velocity_;
  Eigen::VectorXf area_;
  // Voronoi mass matrix
  Eigen::SparseMatrix<float> M_;
  // Per-triangle vectorized Jacobians
  std::vector<Matrix69f> jacobians_;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> cholesky_solver_;

  void init_constrain_permutation();
  Eigen::SparseMatrix<float> init_position_lhs();
  Eigen::SparseMatrix<float> init_bending_lhs();
  Eigen::SparseMatrix<float> init_elastic_lhs();
  bool is_neighboring_face(int f1, int f2);

  // static void rtc_collision_callback(void* data, RTCCollision* collisions,
  //                                    unsigned int num_collisions);

 public:
  RMatrixX3f* pV_ = nullptr;
  RMatrixX3i* pF_ = nullptr;
  std::unordered_set<int>* pconstrain_set = nullptr;

  float position_stiffness = 1e7f;
  float elastic_stiffness_ = 1000.0f;
  float bending_stiffness_ = 1.0f;
  float density_ = 1.0f;
  float dt_ = 1.0f;
  // float collision_thickness_ = 0.01;
  float zero_prune_threshold_ = 1e-8f;
  Eigen::Vector3f constant_acce_field_ = {0.0f, 0.0f, -1.0f};

  int thread_num_ = 4;
  // bool enable_collision = false;

  bool init();
  void reset();
  bool solve();
};
