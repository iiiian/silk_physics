#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

#include "cholmod_utils.hpp"
#include "eigen_alias.hpp"

namespace silk {

/**
 * Static, mesh-dependent quantities used by the cloth solver.
 *
 * Built once from geometry and reused across time steps. This data should not
 * depend on runtime-configurable physical parameters.
 *
 * Notation:
 * vnum = number of vertices.
 * fnum = number of faces.
 * state_num = 3 * vnum.
 */
struct ClothStaticSolverData {
  // Geometry-only Voronoi vertex mass of length vnum (no density applied).
  Eigen::VectorXf mass;
  // Per-face area vector of length fnum.
  Eigen::VectorXf area;
  // Inverse-mass-weighted cotangent matrix for bending energy (vnum x vnum).
  Eigen::SparseMatrix<float> CWC;
  // Area-weighted in-plane elastic matrix (state_num x state_num).
  Eigen::SparseMatrix<float> JWJ;
  // Per-face 6x9 Jacobian operators for in-plane elasticity; order matches
  // TriMesh::F.
  std::vector<Eigen::Matrix<float, 6, 9>> jacobian_ops;
  // Rest curvature per vertex (vnum x 3).
  RMatrixX3f C0;

  // Since Eigen::SparseMatrix lacks noexcept move ctor, explicitly
  // delete copy ctor to avoid error when used in containers like std::vector.
  ClothStaticSolverData() = default;
  ClothStaticSolverData(const ClothStaticSolverData&) = delete;
  ClothStaticSolverData(ClothStaticSolverData&&) = default;
  ClothStaticSolverData& operator=(const ClothStaticSolverData&) = delete;
  ClothStaticSolverData& operator=(ClothStaticSolverData&&) = default;
};

/**
 * Dynamic, time step or config dependent quantities used by the cloth solver.
 *
 * Built at initialization and whenever dt or config changes.
 *
 * Notation:
 * state_num = 3 * vertex num.
 */
struct ClothDynamicSolverData {
  // Time step in seconds.
  float dt;
  // Whether barrier constraints are currently active.
  bool has_barrier_constrain;

  // True vectorized per-vertex mass vector (density-scaled) of length state
  // num.
  Eigen::VectorXf mass;

  // Left-hand side H of Hx = b, combining momentum, bending, in-plane, and pin
  // energies.
  Eigen::SparseMatrix<float> H;

  // Cholesky factorization of H produced via CHOLMOD.
  cholmod_raii::CholmodFactor L;
  // Updated factorization to account for barrier constraints.
  cholmod_raii::CholmodFactor LB;

  // Weighted rest-curvature vector (state_num x 1).
  Eigen::VectorXf C0;
};

}  // namespace silk
