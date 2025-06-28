#include "cloth.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <silk/silk.hpp>
#include <unordered_set>
#include <unsupported/Eigen/KroneckerProduct>
#include <vector>

#include "eigen_helper.hpp"

namespace silk {

bool ClothConfig::is_valid() const {
  if (elastic_stiffness <= 0 || elastic_stiffness > 1000) {
    return false;
  }
  if (bending_stiffness <= 0 || bending_stiffness > 1000) {
    return false;
  }
  if (density <= 0 || density > 1000) {
    return false;
  }
  if (!mesh.is_valid()) {
    return false;
  }
  // pinned vert index should be unique and in range
  uint32_t idx_max = mesh.V.rows();
  std::unordered_set<uint32_t> idx_set;
  for (auto idx : pinned_verts) {
    if (idx >= idx_max || idx_set.count(idx) != 0) {
      return false;
    }
    idx_set.insert(idx);
  }

  return true;
}

ClothElasticConstrain::ClothElasticConstrain(Matrix69f jacobian_op,
                                             Eigen::Vector3i vert_indexes,
                                             uint32_t offset, float weight)
    : jacobian_op_(std::move(jacobian_op)),
      vidx_(std::move(vert_indexes)),
      offset_(offset),
      weight_(weight) {}

void ClothElasticConstrain::project(const Eigen::VectorXf& verts,
                                    Eigen::VectorXf& out) const {
  // assemble local vectorized vertex position
  Eigen::Matrix<float, 9, 1> buffer;
  buffer(Eigen::seqN(0, 3)) = verts(Eigen::seqN(offset_ + 3 * vidx_(0), 3));
  buffer(Eigen::seqN(3, 3)) = verts(Eigen::seqN(offset_ + 3 * vidx_(1), 3));
  buffer(Eigen::seqN(6, 3)) = verts(Eigen::seqN(offset_ + 3 * vidx_(2), 3));

  // deformation matrix
  Eigen::Matrix<float, 3, 2> F = (jacobian_op_ * buffer).reshaped(3, 2);
  // SVD decompose deformation.
  // replacing diagonal term with idenity gives us the projection
  // eigen can't compute thin U and V. so instead compute full UV
  Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
      F, Eigen::ComputeFullV | Eigen::ComputeFullU);
  // this is the projection, deformation F cause by purely rotation +
  // translation
  Eigen::Matrix<float, 3, 2> T =
      svd.matrixU().block<3, 2>(0, 0) * svd.matrixV().transpose();

  // compute the elastic rhs, reuse buffer
  buffer = weight_ * jacobian_op_.transpose() * T.reshaped();
  // add it back to thread local rhs
  out(Eigen::seqN(offset_ + 3 * vidx_(0), 3)) += buffer(Eigen::seqN(0, 3));
  out(Eigen::seqN(offset_ + 3 * vidx_(1), 3)) += buffer(Eigen::seqN(3, 3));
  out(Eigen::seqN(offset_ + 3 * vidx_(2), 3)) += buffer(Eigen::seqN(6, 3));
}

std::optional<Matrix69f> Cloth::vectorized_jacobian_operator(
    Eigen::Ref<const Eigen::Vector3f> v1, Eigen::Ref<const Eigen::Vector3f> v2,
    Eigen::Ref<const Eigen::Vector3f> v3, float zero_threshold) const {
  // convert triangle to 2D
  // use edge e1 = v1 -> v2 as x axis
  // use n x e1 as y axis
  Eigen::Vector3f e1 = v2 - v1;
  Eigen::Vector3f e2 = v3 - v1;

  // basis x
  Eigen::Vector3f bx = e1.normalized();
  // basis y
  Eigen::Vector3f e1xe2 = e1.cross(e2);
  if (e1xe2.norm() < zero_threshold) {
    return std::nullopt;
  }
  Eigen::Vector3f by = e1xe2.cross(e1).normalized();

  // dX is the displacement of the initial triangle in 2D basis
  // dX = ( d1 d2 )
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e1);
  dX(1, 0) = 0;
  dX(0, 1) = bx.dot(e2);
  dX(1, 1) = by.dot(e2);

  // clang-format off
  // D is the displace operator in 3D cartesian basis
  // dx = ( d1 d2 ) = x * D where x = ( v1 v2 v3 )
  const Eigen::Matrix<float, 3, 2> D =
        (Eigen::Matrix<float, 3, 2>() << -1, -1,
                                          1,  0,
                                          0,  1).finished();
  // clang-format on

  // deformation F = dx * (dX)^-1 = x * D * (dX)^-1
  // here we ignore the difference in basis between dx and dX since the
  // additional transformation will be canceled out in later stage of algorithm.
  // use kronecker product to vectorize above equation:
  // given (B^T ⊗  A) vec(X) = vec(AXB),
  // F = x * D * (dX)^-1 = ((D * (dX)^-1)^T ⊗ I3 * vec(x).
  // so the jacobian operator J = ((D * (dX)^-1)^T ⊗ I3
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

Cloth::Cloth(ClothConfig config) : cfg_(std::move(config)) {}

uint32_t Cloth::get_vert_num() const { return cfg_.mesh.V.rows(); }

Eigen::Ref<const Eigen::VectorXf> Cloth::get_init_position() const {
  return cfg_.mesh.V.reshaped<Eigen::RowMajor>();
}

uint32_t Cloth::get_position_offset() const { return solver_position_offset_; }

void Cloth::set_position_offset(uint32_t offset) {
  solver_position_offset_ = offset;
}

Eigen::Ref<const Eigen::VectorXi> Cloth::get_pinned_verts() const {
  return cfg_.pinned_verts;
}

SolverInitData Cloth::compute_solver_init_data() const {
  const Mesh& mesh = cfg_.mesh;
  int vnum = mesh.V.rows();
  int fnum = mesh.F.rows();
  SolverInitData init_data;

  // vertex mass based on voroni area
  Eigen::SparseMatrix<float> mass;
  igl::massmatrix(mesh.V, mesh.F, igl::MASSMATRIX_TYPE_VORONOI, mass);
  mass *= cfg_.density;
  vectorize_sparse_to_triplets(mass, init_data.mass, solver_position_offset_,
                               solver_position_offset_);
  //
  // // cotangent laplacian with inverse mass as weight
  // Eigen::SparseMatrix<float> C;  // cotangent matrix
  // igl::cotmatrix(mesh.V, mesh.F, C);
  // Eigen::SparseMatrix<float> W(vnum, vnum);  // cotangent laplacian weight
  // W.setIdentity();
  // for (int i = 0; i < vnum; i++) {
  //   W.coeffRef(i, i) = 1 / mass.coeff(i, i);
  // }
  // // this is the weighted AA for bending energy.
  // // assume initial curvature is 0 so there is no solver constrain for
  // bending Eigen::SparseMatrix<float> CWC =
  //     cfg_.bending_stiffness * C.transpose() * W * C;
  // vectorize_sparse_to_triplets(CWC, init_data.weighted_AA,
  //                              solver_position_offset_,
  //                              solver_position_offset_);
  //
  // // triangle area
  // Eigen::VectorXf area;
  // igl::doublearea(mesh.V, mesh.F, area);
  // area /= 2;
  //
  // // in-plane deformation
  // for (int f = 0; f < fnum; ++f) {
  //   auto vidx = mesh.F.row(f);
  //   auto jacobian_op = vectorized_jacobian_operator(
  //       mesh.V.row(vidx(0)), mesh.V.row(vidx(1)), mesh.V.row(vidx(2)));
  //   if (!jacobian_op) {
  //     // TODO: handle degenerate triangle better
  //     // SPDLOG_WARN("degenerate triangle {}", f);
  //     continue;
  //   }
  //
  //   float weight = cfg_.elastic_stiffness * area(f);
  //   init_data.constrains.emplace_back(std::make_unique<ClothElasticConstrain>(
  //       *jacobian_op, vidx, solver_position_offset_, weight));
  //   Eigen::Matrix<float, 9, 9> local_AA =
  //       (*jacobian_op).transpose() * (*jacobian_op);
  //
  //   // convert the local AA to global AA
  //   for (int vi = 0; vi < 3; ++vi) {
  //     for (int vj = 0; vj < 3; ++vj) {
  //       for (int i = 0; i < 3; ++i) {
  //         for (int j = 0; j < 3; ++j) {
  //           float val = weight * local_AA(3 * vi + i, 3 * vj + j);
  //           if (std::abs(val) == 0) {
  //             continue;
  //           }
  //           // TODO: consider purging value near zero to save compute
  //           init_data.weighted_AA.emplace_back(
  //               solver_position_offset_ + 3 * vidx(vi) + i,
  //               solver_position_offset_ + 3 * vidx(vj) + j, val);
  //         }
  //       }
  //     }
  //   }
  // }
  //
  // // pinned vertices
  // if (cfg_.pinned_verts.size() != 0) {
  //   // TODO: remove hard coded position constrain stiffness
  //   for (auto idx : cfg_.pinned_verts) {
  //     uint32_t offset = solver_position_offset_ + 3 * idx;
  //     init_data.weighted_AA.emplace_back(offset, offset, 1e6);
  //     init_data.weighted_AA.emplace_back(offset + 1, offset + 1, 1e6);
  //     init_data.weighted_AA.emplace_back(offset + 2, offset + 2, 1e6);
  //   }
  //   init_data.constrains.emplace_back(std::make_unique<PositionConstrain>(
  //       cfg_.pinned_verts, solver_position_offset_, 1e6));
  // }

  return init_data;
}

}  // namespace silk
