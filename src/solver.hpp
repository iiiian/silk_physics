#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <unordered_set>
#include <vector>

#include "common_types.hpp"

class ClothSolver {
 private:
  Eigen::VectorXf velocity_;
  Eigen::VectorXf area_;
  // Voronoi mass matrix
  Eigen::SparseMatrix<float> M_;
  // Per-triangle vectorized Jacobians
  std::vector<Matrix69f> jacobians_;
  std::vector<Eigen::Triplet<float>> AA_triplets_;

  int knum_ = 0;
  int unum_ = 0;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm_;

  // Bending constraints
  Eigen::SparseMatrix<float> Guk_;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> cholesky_solver_;

  bool init_elastic_constrain();

 public:
  RMatrixX3f* pV_ = nullptr;
  Eigen::MatrixX3i* pF_ = nullptr;
  std::unordered_set<int>* pconstrain_set = nullptr;
  RMatrixX3f* pCV_ = nullptr;

  float elastic_stiffness_ = 1.0f;
  float bending_stiffness_ = 1.0f;
  float density_ = 1.0f;
  float dt_ = 1.0f;
  float zero_prune_threshold_ = 1e-8f;
  Eigen::Vector3f constant_force_field_ = {0.0f, 0.0f, -1.0f};

  bool init();
  void reset();
  bool solve();
};
