#include "cloth_topology.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <silk/silk.hpp>
#include <unsupported/Eigen/KroneckerProduct>
#include <vector>

#include "eigen_alias.hpp"
#include "logger.hpp"
#include "mesh.hpp"

namespace silk {

/** Compute the Jacobian operator that maps stacked vertex positions of a
 *  triangle (9x1) to the 3x2 deformation gradient in a local 2D chart, and
 *  return it in vectorized form as a 6x9 matrix J such that vec(F) = J * x.
 *
 *  Preconditions: the triangle is non‑degenerate (non‑zero edge length and
 *  area). Basis differences between dx and dX are ignored intentionally since
 *  they cancel out in the quadratic form used later.
 */
Eigen::Matrix<float, 6, 9> triangle_jacobian_operator(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2) {
  // Convert triangle to 2D.
  // Use edge e1 = v1 -> v2 as x axis.
  // Use n x e1 as y axis.
  Eigen::Vector3f e0 = v1 - v0;
  Eigen::Vector3f e1 = v2 - v0;

  Eigen::Vector3f e0xe1 = e0.cross(e1);
  // We assume no degenerate triangle exists.
  float area2_eps =
      std::pow(1e-6f * std::max(e0.squaredNorm(), e1.squaredNorm()), 2);
  if (e0xe1.squaredNorm() < area2_eps) {
    SPDLOG_DEBUG("degenerate triangle in cloth mesh");
  }
  // Define 2D basis x and y.
  Eigen::Vector3f bx = e0.normalized();
  Eigen::Vector3f by = e0xe1.cross(e0).normalized();

  // The matrix dX is the displacement of the initial triangle in a 2D basis.
  // Definition: dX = ( d1 d2 ).
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e0);
  dX(1, 0) = 0.0f;
  dX(0, 1) = bx.dot(e1);
  dX(1, 1) = by.dot(e1);

  // clang-format off
  // D is the displacement operator in 3D Cartesian basis.
  // The matrix dx equals ( d1 d2 ) = x * D where x = ( v1 v2 v3 ).
  const Eigen::Matrix<float, 3, 2> D =
        (Eigen::Matrix<float, 3, 2>() << -1, -1,
                                          1,  0,
                                          0,  1).finished();
  // clang-format on

  // Deformation: F = dx * (dX)^-1 = x * D * (dX)^-1.
  // We vectorize via the Kronecker identity (B^T ⊗ A) vec(X) = vec(AXB).
  // We have vec(F) = ((D * (dX)^-1)^T ⊗ I3) vec(x) := J vec(x).
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

ClothTopology::ClothTopology(const ClothConfig& config, const TriMesh& mesh) {
  const TriMesh& m = mesh;

  int vert_num = m.V.rows();
  int face_num = m.F.rows();

  Eigen::SparseMatrix<float> voroni_mass;
  igl::massmatrix(m.V, m.F, igl::MASSMATRIX_TYPE_VORONOI, voroni_mass);
  mass = voroni_mass.diagonal();

  // Cotangent matrix.
  Eigen::SparseMatrix<float> C;
  igl::cotmatrix(m.V, m.F, C);

  // Cotangent Laplacian weight.
  Eigen::SparseMatrix<float> W(vert_num, vert_num);
  W.setIdentity();
  for (int i = 0; i < vert_num; ++i) {
    W.coeffRef(i, i) = 1.0f / voroni_mass.coeff(i, i);
  }

  // Bending energy.
  CWC = C.transpose() * W * C;
  C0 = CWC * m.V;

  // Triangle area.
  igl::doublearea(m.V, m.F, area);
  area /= 2;

  // In‑plane deformation energy.
  std::vector<Eigen::Triplet<float>> JWJ_triplets;
  for (int f = 0; f < face_num; ++f) {
    Eigen::Matrix<float, 6, 9> jop = triangle_jacobian_operator(
        m.V.row(m.F(f, 0)), m.V.row(m.F(f, 1)), m.V.row(m.F(f, 2)));

    float weight = area(f);
    Eigen::Matrix<float, 9, 9> local_AA = weight * jop.transpose() * jop;
    jacobian_ops.push_back(jop);

    // Scatter local AA into the global (vectorized-vertex) system.
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            float val = local_AA(3 * vi + i, 3 * vj + j);
            JWJ_triplets.emplace_back(3 * m.F(f, vi) + i, 3 * m.F(f, vj) + j,
                                      val);
          }
        }
      }
    }
  }

  JWJ.resize(3 * vert_num, 3 * vert_num);
  JWJ.setFromTriplets(JWJ_triplets.begin(), JWJ_triplets.end());
}

}  // namespace silk
