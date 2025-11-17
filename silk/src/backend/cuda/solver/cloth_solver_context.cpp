#include "backend/cuda/solver/cloth_solver_context.hpp"

#include <Eigen/Core>

#include "backend/cuda/cuda_utils.hpp"
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
                                       const Pin& pin, float dt) {
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
  append_triplets_from_sparse(t.JWJ, 0, 0, c.elastic_stiffness, H_triplets,
                              Symmetry::Upper);
  append_triplets_from_vectorized_sparse(t.CWC, 0, 0, c.bending_stiffness,
                                         H_triplets, Symmetry::Upper);
  for (int i = 0; i < pin.index.size(); ++i) {
    int offset = 3 * pin.index(i);
    H_triplets.emplace_back(offset, offset, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 1, offset + 1, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 2, offset + 2, pin.pin_stiffness);
  }

  // Create diagonal part D and off-diagonal part R.
  Eigen::SparseMatrix<float, Eigen::RowMajor> R(state_num, state_num);
  Eigen::VectorXf D(state_num);
  for (auto& triplet : H_triplets) {
    if (triplet.row() == triplet.col()) {
      D(triplet.row()) = triplet.value();
    } else {
      R.coeffRef(triplet.row(), triplet.col()) = triplet.value();
    }
  }
  R.makeCompressed();

  Eigen::SparseMatrix<float, Eigen::RowMajor> RR = R * R;

  std::vector<float> jacobian_ops(t.jacobian_ops.size() * 54);
  for (int i = 0; i < t.jacobian_ops.size(); ++i) {
    memcpy(jacobian_ops.data() + 54 * i, t.jacobian_ops[i].data(),
           54 * sizeof(float));
  }

  this->dt = dt;
  this->state_num = state_num;
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
  this->d_R = CSRMatrix{R};
  this->d_RR = CSRMatrix{RR};
  this->d_F = host_eigen_to_device(mesh.F);
  this->d_jacobian_ops = host_vector_to_device(jacobian_ops);
  this->d_C0 = host_eigen_to_device(t.C0);
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
  std::swap(d_RR, other.d_RR);
  std::swap(d_F, other.d_F);
  std::swap(d_jacobian_ops, other.d_jacobian_ops);
  std::swap(d_C0, other.d_C0);
}

}  // namespace silk::cuda
