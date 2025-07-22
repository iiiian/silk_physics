#include "make_solver_data.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <memory>
#include <optional>
#include <unsupported/Eigen/KroneckerProduct>

#include "ecs.hpp"
#include "eigen_helper.hpp"
#include "solver_constrain.hpp"

namespace silk {

class ClothElasticConstrain : public ISolverConstrain {
 private:
  Eigen::Matrix<float, 6, 9> jacobian_op_;
  Eigen::Vector3i offset_;  // vertex offsets
  float weight_;

 public:
  ClothElasticConstrain(Eigen::Matrix<float, 6, 9> jacobian_op,
                        Eigen::Vector3i offset, float weight)
      : jacobian_op_(jacobian_op), offset_(offset), weight_(weight) {}

  // impl solver constrain interface
  void project(const Eigen::VectorXf& solver_state,
               Eigen::VectorXf& out) const override {
    // assemble local vectorized vertex position
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = solver_state(Eigen::seqN(offset_(0), 3));
    buffer(Eigen::seqN(3, 3)) = solver_state(Eigen::seqN(offset_(1), 3));
    buffer(Eigen::seqN(6, 3)) = solver_state(Eigen::seqN(offset_(2), 3));

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
    out(Eigen::seqN(offset_(0), 3)) += buffer(Eigen::seqN(0, 3));
    out(Eigen::seqN(offset_(1), 3)) += buffer(Eigen::seqN(3, 3));
    out(Eigen::seqN(offset_(2), 3)) += buffer(Eigen::seqN(6, 3));
  }
};

std::optional<Eigen::Matrix<float, 6, 9>> cloth_jacobian_operator(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2, float zero_threshold) {
  // convert triangle to 2D
  // use edge e1 = v1 -> v2 as x axis
  // use n x e1 as y axis
  Eigen::Vector3f e0 = v1 - v0;
  Eigen::Vector3f e1 = v2 - v0;

  // basis x
  Eigen::Vector3f bx = e0.normalized();
  // basis y
  Eigen::Vector3f e0xe1 = e0.cross(e1);
  if (e0xe1.norm() < zero_threshold) {
    return std::nullopt;
  }
  Eigen::Vector3f by = e0xe1.cross(e0).normalized();

  // dX is the displacement of the initial triangle in 2D basis
  // dX = ( d1 d2 )
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e0);
  dX(1, 0) = 0;
  dX(0, 1) = bx.dot(e1);
  dX(1, 1) = by.dot(e1);

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

SolverData make_cloth_solver_data(const ClothConfig& config,
                                  const TriMesh& tri_mesh,
                                  const PinnedGroup& pinned_group,
                                  int solver_offset) {
  const ClothConfig& c = config;
  const TriMesh& m = tri_mesh;

  int vert_num = m.V.rows();
  int face_num = m.F.rows();

  std::vector<Eigen::Triplet<float>> weighted_AA;
  std::vector<std::unique_ptr<ISolverConstrain>> constrains;

  Eigen::SparseMatrix<float> mass;
  igl::massmatrix(m.V, m.F, igl::MASSMATRIX_TYPE_VORONOI, mass);
  mass *= c.density;

  // cotangent matrix
  Eigen::SparseMatrix<float> C;
  igl::cotmatrix(m.V, m.F, C);

  // cotangent laplacian weight
  Eigen::SparseMatrix<float> W(vert_num, vert_num);
  W.setIdentity();
  for (int i = 0; i < vert_num; ++i) {
    W.coeffRef(i, i) = 1.0f / mass.coeff(i, i);
  }

  // this is the weighted AA for bending energy.
  // assume initial curvature is 0 so there is no solver constrain for bending
  Eigen::SparseMatrix<float> CWC = c.bending_stiffness * C.transpose() * W * C;
  vectorize_sparse_to_triplets(CWC, weighted_AA, solver_offset, solver_offset);

  // triangle area
  Eigen::VectorXf area;
  igl::doublearea(m.V, m.F, area);
  area /= 2;

  // in-plane deformation energy
  for (int f = 0; f < face_num; ++f) {
    // TODO: avoid hardcoded jacobian zero_threshold
    auto jop = cloth_jacobian_operator(m.V.row(m.F(f, 0)), m.V.row(m.F(f, 1)),
                                       m.V.row(m.F(f, 2)), 0.0f);
    if (!jop) {
      // TODO: handle degenerate triangle better?
      continue;
    }

    float weight = c.elastic_stiffness * area(f);
    Eigen::Vector3i offset = solver_offset + 3 * m.F.row(f).array();
    constrains.emplace_back(
        std::make_unique<ClothElasticConstrain>(*jop, offset, weight));
    Eigen::Matrix<float, 9, 9> local_AA = (*jop).transpose() * (*jop);

    // convert local AA to global AA
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            float val = weight * local_AA(3 * vi + i, 3 * vj + j);
            // TODO: avoid hardcoded zero zero threshold
            if (std::abs(val) == 0.0f) {
              continue;
            }
            // TODO: consider purging value near zero to save compute
            weighted_AA.emplace_back(3 * m.F(f, vi) + i, 3 * m.F(f, vj) + j,
                                     val);
          }
        }
      }
    }
  }

  // pinned vertices
  if (pinned_group.pinnned_index.size() != 0) {
    // TODO: remove hard coded position constrain stiffness
    for (int idx : pinned_group.pinnned_index) {
      int offset = solver_offset + 3 * idx;
      weighted_AA.emplace_back(offset, offset, 1e6f);
      weighted_AA.emplace_back(offset + 1, offset + 1, 1e6f);
      weighted_AA.emplace_back(offset + 2, offset + 2, 1e6f);
    }
  }

  SolverData data;
  data.offset = solver_offset;
  data.mass.resize(vert_num);
  for (int i = 0; i < vert_num; ++i) {
    data.mass(i) = mass.coeff(i, i);
  }
  data.weighted_AA = std::move(weighted_AA);
  data.constrains = std::move(constrains);

  return data;
}

void make_all_solver_data(Registry& registry) {
  int offset_counter = 0;
  for (Entity& e : registry.entity.data()) {
    // remove old solver data
    ECS_REMOVE(registry, e, solver_data);

    auto cloth_config = ECS_GET(registry, e, cloth_config);
    auto tri_mesh = ECS_GET(registry, e, tri_mesh);
    auto pinned_group = ECS_GET(registry, e, pinned_group);

    if (cloth_config && tri_mesh && pinned_group) {
      Handle h = registry.solver_data.add(make_cloth_solver_data(
          *cloth_config, *tri_mesh, *pinned_group, offset_counter));
      assert(!h.is_empty());
      e.solver_data = h;

      offset_counter += 3 * tri_mesh->V.rows();
      continue;
    }
  }
}

}  // namespace silk
