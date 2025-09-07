#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

#include "cholmod_utils.hpp"
#include "eigen_alias.hpp"

namespace silk {

struct ClothStaticSolverData {
  // vnum means vertex num
  // fnum means face num

  // vertex mass. dimension vnum x vnum
  Eigen::VectorXf mass;
  // face area. dimension fnum x 1
  Eigen::VectorXf area;
  // inv mass weighted cotangent matrix for bending energy.
  // dimension vnum x vnum.
  Eigen::SparseMatrix<float> CWC;
  // face area weighted jabobian matrix for in-plane elastic energy.
  // dimension state_num x state_num.
  Eigen::SparseMatrix<float> JWJ;
  // jacobian operator of triangle faces for in-plane elastic energy.
  // the order is the same as the face order of TriMesh F matrix.
  std::vector<Eigen::Matrix<float, 6, 9>> jacobian_ops;
  // rest curvature. dimension  vnum x 3
  RMatrixX3f C0;

  // Since Eigen::SparseMatrix lacks noexcept move ctor, we have to explicitly
  // delete copy ctor to avoid error when used in containers like std::vector.
  ClothStaticSolverData() = default;
  ClothStaticSolverData(ClothStaticSolverData&) = delete;
  ClothStaticSolverData(ClothStaticSolverData&&) = default;
  ClothStaticSolverData& operator=(ClothStaticSolverData&) = delete;
  ClothStaticSolverData& operator=(ClothStaticSolverData&&) = default;
};

struct ClothDynamicSolverData {
  float dt;
  bool has_barrier_constrain;

  // Lhs of linear system Hx = b. Contains momentum energy, bending energy,
  // in-plane elastic energy and pin energy.
  Eigen::SparseMatrix<float> H;

  // cholesky factorization of H, which is the lhs of final system equation
  // Hx = b. assembled from momentum energy, in-plane elastic energy, bending
  // energy, and pin.
  cholmod_raii::CholmodFactor L;
  // updated L to account for barrier constrains
  cholmod_raii::CholmodFactor LB;
};

}  // namespace silk
