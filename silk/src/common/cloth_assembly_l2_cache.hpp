#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

#include "common/eigen_alias.hpp"
#include "common/mesh.hpp"
#include "silk/silk.hpp"

namespace silk {

/// Static, mesh-dependent quantities used for cloth simulation.
///
/// Built once from geometry and reused across time steps. This data should not
/// depend on runtime-configurable physical parameters.
///
/// Notation:
/// vnum = number of vertices.
/// fnum = number of faces.
/// state_num = 3 * vnum.
class ClothAssemblyL2Cache {
 public:
  // Voronoi vertex mass of length vnum (no density applied).
  Eigen::VectorXf mass;

  // Per-face area vector of length fnum.
  Eigen::VectorXf area;

  // Faces (fnum x 3).
  RMatrixX3i F;

  // Inverse-mass-weighted cotangent matrix for bending energy (vnum x vnum).
  Eigen::SparseMatrix<float> CWC;

  // Cotangent matrix to compute curvature.
  Eigen::SparseMatrix<float> laplacian_ops;

  // Rest curvature per vertex (vnum x 3).
  RMatrixX3f C0;

  // Area-weighted in-plane elastic matrix (state_num x state_num).
  Eigen::SparseMatrix<float> JWJ;

  // Per-face 6x9 Jacobian operators for in-plane elasticity.
  std::vector<Eigen::Matrix<float, 6, 9>> jacobian_ops;

 public:
  ClothAssemblyL2Cache() = default;
  ClothAssemblyL2Cache(const ClothConfig& config, const TriMesh& mesh);

  // Since Eigen::SparseMatrix lacks noexcept move ctor, explicitly
  // delete copy ctor to avoid error in containers like std::vector.
  ClothAssemblyL2Cache(const ClothAssemblyL2Cache&) = delete;
  ClothAssemblyL2Cache(ClothAssemblyL2Cache&&) = default;
  ClothAssemblyL2Cache& operator=(const ClothAssemblyL2Cache&) = delete;
  ClothAssemblyL2Cache& operator=(ClothAssemblyL2Cache&&) = default;
};

}  // namespace silk
