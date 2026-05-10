#include <Eigen/Core>

// #include "backend/cuda/copy_vector_like.hpp"
#include "backend/cuda/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/compute_subspace_u.hpp"
#include "backend/cuda/sparse_matrix_interop.hpp"
#include "common/cloth_assembly_l2_cache.hpp"
#include "common/eigen_utils.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

ClothAssemblyL1Cache::ClothAssemblyL1Cache(const ClothConfig& config,
                                           const ClothAssemblyL2Cache& topology,
                                           float dt, CudaRuntime rt) {
  auto& c = config;
  auto& t = topology;
  int state_num = 3 * t.mass.size();

  // Assemble H matrix.
  std::vector<Eigen::Triplet<float>> H_triplets;
  // Inertia term.
  Eigen::VectorXf M = 1.0f / (dt * dt) * c.density * t.mass;
  for (int i = 0; i < M.size(); ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }
  // Elastic term.
  append_triplets_from_sparse(t.JWJ, 0, 0, c.elastic_stiffness, H_triplets);
  // Bending term.
  append_triplets_from_vectorized_sparse(t.CWC, 0, 0, c.bending_stiffness,
                                         H_triplets);
  Eigen::SparseMatrix<float> h_H{state_num, state_num};
  h_H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  // Assemble jacobian ops.
  std::vector<float> h_jacobian_ops(t.jacobian_ops.size() * 54);
  for (int i = 0; i < t.jacobian_ops.size(); ++i) {
    memcpy(h_jacobian_ops.data() + 54 * i, t.jacobian_ops[i].data(),
           54 * sizeof(float));
  }

  // Assemble mass.
  std::vector<float> h_mass(state_num);
  for (int i = 0; i < t.mass.size(); ++i) {
    float val = c.density * t.mass(i);
    h_mass[3 * i] = val;
    h_mass[3 * i + 1] = val;
    h_mass[3 * i + 2] = val;
  }

  this->dt = dt;
  this->state_num = state_num;
  this->mass = vec_like_to_device<float>(h_mass, rt);
  this->area = host_eigen_to_device(t.area, rt);
  this->jacobian_ops = vec_like_to_device<float>(h_jacobian_ops, rt);
  this->C0 = host_eigen_to_device(
      (c.bending_stiffness * t.C0).reshaped<Eigen::RowMajor>(), rt);
  this->SS_sum = BSRMatrix{h_H, 3, {}, rt};
}

}  // namespace silk::cuda
