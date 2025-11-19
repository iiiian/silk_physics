#include "backend/cuda/solver/cloth_solver_context.hpp"

#include <Eigen/Core>

#include "backend/cuda/copy_vector_like.hpp"
#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/solver/compute_subspace_u.hpp"
#include "backend/cuda/sparse_matrix_interop.hpp"
#include "common/cloth_topology.hpp"
#include "common/eigen_utils.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

ClothSolverContext::ClothSolverContext(const ClothConfig& config,
                                       const TriMesh& mesh,
                                       const ClothTopology& topology,
                                       const Pin& pin, const ObjectState& state,
                                       float dt) {
  auto& c = config;
  auto& t = topology;
  int state_num = 3 * t.mass.size();

  // Assemble H matrix.
  std::vector<Eigen::Triplet<float>> H_triplets;
  Eigen::VectorXf M = 1.0f / (dt * dt) * c.density * t.mass;
  for (int i = 0; i < M.size(); ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }
  append_triplets_from_sparse(t.JWJ, 0, 0, c.elastic_stiffness, H_triplets);
  append_triplets_from_vectorized_sparse(t.CWC, 0, 0, c.bending_stiffness,
                                         H_triplets);
  for (int i = 0; i < pin.index.size(); ++i) {
    int v_old = pin.index(i);
    int v_new = state.inv_perm(v_old);
    int offset = 3 * v_new;
    H_triplets.emplace_back(offset, offset, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 1, offset + 1, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 2, offset + 2, pin.pin_stiffness);
  }

  // Create diagonal part D and off-diagonal part R.
  Eigen::SparseMatrix<float, Eigen::RowMajor> R(state_num, state_num);
  Eigen::VectorXf D = Eigen::VectorXf::Zero(state_num);
  for (auto& triplet : H_triplets) {
    if (triplet.row() == triplet.col()) {
      D(triplet.row()) += triplet.value();
    } else {
      // Accumulate off-diagonal contributions instead of overwriting.
      R.coeffRef(triplet.row(), triplet.col()) += triplet.value();
    }
  }
  R.makeCompressed();

  std::vector<float> jacobian_ops(t.jacobian_ops.size() * 54);
  for (int i = 0; i < t.jacobian_ops.size(); ++i) {
    memcpy(jacobian_ops.data() + 54 * i, t.jacobian_ops[i].data(),
           54 * sizeof(float));
  }

  Eigen::SparseMatrix<float> H{state_num, state_num};
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());
  // Hard code subspace dim and assumes eigen decomposition success here for
  // now.
  // TODO: configurable subspace dim and propagate error properly.
  auto U = compute_subspace_u(H, 32);
  assert(U.has_value());
  Eigen::VectorXf HX = H * mesh.V.reshaped<Eigen::RowMajor>();

  this->dt = dt;
  this->state_num = state_num;
  this->face_num = mesh.F.rows();
  this->h_mass.resize(state_num);
  for (int i = 0; i < t.mass.size(); ++i) {
    float val = c.density * t.mass(i);
    h_mass(3 * i) = val;
    h_mass(3 * i + 1) = val;
    h_mass(3 * i + 2) = val;
  }
  this->d_mass = host_eigen_to_device(h_mass);
  this->d_area = host_eigen_to_device(t.area);
  this->d_D = host_eigen_to_device(D);
  this->d_DB = host_eigen_to_device(D);
  this->d_R = make_csr_from_eigen(R);
  this->d_F = host_eigen_to_device(mesh.F);
  this->d_jacobian_ops = host_vector_to_device(jacobian_ops);
  this->d_C0 = host_eigen_to_device(
      (c.bending_stiffness * t.C0).reshaped<Eigen::RowMajor>());
  this->r = 32;
  this->UHU = (*U).transpose() * H * (*U);
  this->d_U = host_eigen_to_device(*U);
  // Row‑major copy of U for kernels that traverse rows.
  using MatrixXfRowMajor =
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  MatrixXfRowMajor U_row_major = (*U);
  this->d_U_RM = host_eigen_to_device(U_row_major);
  this->d_HX = host_eigen_to_device(HX);
  this->d_X = host_eigen_to_device(mesh.V.reshaped<Eigen::RowMajor>());
}

ClothSolverContext::ClothSolverContext(ClothSolverContext&& other) noexcept {
  swap(other);
}

ClothSolverContext& ClothSolverContext::operator=(
    ClothSolverContext&& other) noexcept {
  swap(other);
  return *this;
}

ClothSolverContext::~ClothSolverContext() {
  CHECK_CUDA(cudaFree(d_mass));
  CHECK_CUDA(cudaFree(d_area));
  CHECK_CUDA(cudaFree(d_D));
  CHECK_CUDA(cudaFree(d_DB));
  CHECK_CUDA(cudaFree(d_F));
  CHECK_CUDA(cudaFree(d_jacobian_ops));
  CHECK_CUDA(cudaFree(d_C0));
  CHECK_CUDA(cudaFree(d_U));
  CHECK_CUDA(cudaFree(d_U_RM));
  CHECK_CUDA(cudaFree(d_HX));
  CHECK_CUDA(cudaFree(d_X));
}

void ClothSolverContext::swap(ClothSolverContext& other) noexcept {
  if (this == &other) {
    return;
  }

  std::swap(dt, other.dt);
  std::swap(state_num, other.state_num);
  std::swap(face_num, other.face_num);
  std::swap(h_mass, other.h_mass);
  std::swap(d_mass, other.d_mass);
  std::swap(d_area, other.d_area);
  std::swap(d_D, other.d_D);
  std::swap(d_DB, other.d_DB);
  std::swap(d_R, other.d_R);
  std::swap(d_F, other.d_F);
  std::swap(d_jacobian_ops, other.d_jacobian_ops);
  std::swap(d_C0, other.d_C0);
  std::swap(r, other.r);
  std::swap(UHU, other.UHU);
  std::swap(d_U, other.d_U);
  std::swap(d_U_RM, other.d_U_RM);
  std::swap(d_HX, other.d_HX);
  std::swap(d_X, other.d_X);
}

}  // namespace silk::cuda
