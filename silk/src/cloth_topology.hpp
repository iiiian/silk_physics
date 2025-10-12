#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <silk/silk.hpp>
#include <vector>

#include "eigen_alias.hpp"
#include "mesh.hpp"

namespace silk {

/**
 * Static, mesh-dependent quantities used for cloth simulation.
 *
 * Built once from geometry and reused across time steps. This data should not
 * depend on runtime-configurable physical parameters.
 *
 * Notation:
 * vnum = number of vertices.
 * fnum = number of faces.
 * state_num = 3 * vnum.
 */
class ClothTopology {
 public:
  // Voronoi vertex mass of length vnum (no density applied).
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

 public:
  ClothTopology(const ClothConfig& config, const TriMesh& mesh);

  // Since Eigen::SparseMatrix lacks noexcept move ctor, explicitly
  // delete copy ctor to avoid error when used in containers like std::vector.
  ClothTopology(const ClothTopology&) = delete;
  ClothTopology(ClothTopology&&) = default;
  ClothTopology& operator=(const ClothTopology&) = delete;
  ClothTopology& operator=(ClothTopology&&) = default;
};

}  // namespace silk
