#include "cloth.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <spdlog/spdlog.h>

#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <unsupported/Eigen/KroneckerProduct>
#include <vector>

#include "api.hpp"
#include "eigen_helper.hpp"
#include "mesh.hpp"
#include "physical_body.hpp"

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
  return true;
}

ClothSolverConstrain::ClothSolverConstrain(Matrix69f jacobian_op,
                                           Eigen::Vector3i vert_indexes,
                                           float weight)
    : jacobian_op_(std::move(jacobian_op)),
      vidx_(std::move(vert_indexes)),
      weight_(weight) {}

void ClothSolverConstrain::project(uint32_t vert_offset,
                                   const Eigen::VectorXf& verts,
                                   Eigen::VectorXf& out) const {
  // assemble local vectorized vertex position
  Eigen::Matrix<float, 9, 1> buffer;
  buffer(Eigen::seqN(0, 3)) = verts(Eigen::seqN(3 * vidx_(0), 3));
  buffer(Eigen::seqN(3, 3)) = verts(Eigen::seqN(3 * vidx_(1), 3));
  buffer(Eigen::seqN(6, 3)) = verts(Eigen::seqN(3 * vidx_(2), 3));

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
  out(Eigen::seqN(3 * vidx_(0), 3)) += buffer(Eigen::seqN(0, 3));
  out(Eigen::seqN(3 * vidx_(1), 3)) += buffer(Eigen::seqN(3, 3));
  out(Eigen::seqN(3 * vidx_(2), 3)) += buffer(Eigen::seqN(6, 3));
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
  // here we ignore the differnt basis of dx and dX since the additional
  // transformation will be canceled out in later stage of algorithm.
  // use kronecker product to vectorize above equation:
  // given (B^T ⊗  A) vec(X) = vec(AXB),
  // F = x * D * (dX)^-1 = ((D * (dX)^-1)^T ⊗ I3 * vec(x).
  // so the jacobian operator J = ((D * (dX)^-1)^T ⊗ I3
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

Cloth::Cloth(Mesh mesh, ClothConfig config)
    : mesh_(std::move(mesh)), config_(std::move(config)) {}

SolverInitData Cloth::compute_solver_init_data() const {
  assert(mesh_.is_valid());
  assert(config_.is_valid());

  int vnum = mesh_.V.rows();
  int fnum = mesh_.F.rows();
  SolverInitData init_data;

  // vertex mass based on voroni area
  Eigen::SparseMatrix<float> mass;
  igl::massmatrix(mesh_.V, mesh_.F, igl::MASSMATRIX_TYPE_VORONOI, mass);
  mass *= config_.density;
  vectorize_sparse_to_triplets(mass, init_data.mass);

  // cotangent laplacian with inverse mass as weight
  Eigen::SparseMatrix<float> C;  // cotangent matrix
  igl::cotmatrix(mesh_.V, mesh_.F, C);
  Eigen::SparseMatrix<float> W(vnum, vnum);  // cotangent laplacian weight
  W.setIdentity();
  for (int i = 0; i < vnum; i++) {
    W.coeffRef(i, i) = 1 / mass.coeff(i, i);
  }
  // this is the weighted AA for bending energy.
  // assume initial curvature is 0 so there is no solver constrain for bending
  Eigen::SparseMatrix<float> CWC =
      config_.bending_stiffness * C.transpose() * W * C;
  vectorize_sparse_to_triplets(CWC, init_data.weighted_AA);

  // triangle area
  Eigen::VectorXf area;
  igl::doublearea(mesh_.V, mesh_.F, area);
  area /= 2;

  // in-plane deformation
  for (int f = 0; f < fnum; ++f) {
    auto vidx = mesh_.F.row(f);
    auto jacobian_op = vectorized_jacobian_operator(
        mesh_.V.row(vidx(0)), mesh_.V.row(vidx(1)), mesh_.V.row(vidx(2)));
    if (!jacobian_op) {
      // TODO: handle degenerate triangle better
      spdlog::warn("degenerate triangle {}", f);
      continue;
    }

    float weight = config_.elastic_stiffness * area(f);
    init_data.constrains.emplace_back(
        std::make_unique<ClothSolverConstrain>(*jacobian_op, vidx, weight));
    Eigen::Matrix<float, 9, 9> local_AA =
        (*jacobian_op).transpose() * (*jacobian_op);

    // convert the local AA to global AA
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            float val = weight * local_AA(3 * vi + i, 3 * vj + j);
            // TODO: zero prune threshold
            // if (abs(val) < zero_prune_threshold_) {
            //   continue;
            // }
            init_data.weighted_AA.emplace_back(3 * vidx(vi) + i,
                                               3 * vidx(vj) + j, val);
          }
        }
      }
    }
  }

  return init_data;
}

uint32_t Cloth::get_vert_num() const { return mesh_.V.rows(); }

const RMatrixX3f& Cloth::get_init_position() const { return mesh_.V; }

uint32_t Cloth::get_position_offset() const { return solver_position_offset_; }

void Cloth::set_position_offset(uint32_t offset) {
  solver_position_offset_ = offset;
}
