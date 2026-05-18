#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"

#include <Eigen/Core>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "common/cloth_assembly_l2_cache.hpp"
#include "common/eigen_utils.hpp"
#include "common/logger.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

ClothAssemblyL1Cache::ClothAssemblyL1Cache(const ClothConfig& config,
                                           const MeshPartition& partition,
                                           const ClothAssemblyL2Cache& l2_cache,
                                           float dt, CudaRuntime rt) {
  auto& c = config;
  auto& l2 = l2_cache;
  int state_num = 3 * l2.mass.size();

  // TODO: Experiements with penalty.
  constexpr float PENALTY = 1.0;

  // Assemble H matrix.
  std::vector<Eigen::Triplet<float>> AA_triplets;
  // Elastic term.
  append_triplets_from_sparse(l2.JWJ, 0, 0, 1.0f, AA_triplets);
  // Bending term.
  append_triplets_from_vectorized_sparse(l2.CWC, 0, 0, c.bending_stiffness,
                                         AA_triplets);
  Eigen::SparseMatrix<float> h_AA{state_num, state_num};
  h_AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());
  h_AA *= PENALTY;

  // Assemble vectorized Laplacian operator.
  Eigen::SparseMatrix<float> h_weighted_laplacian =
      l2.mass.asDiagonal() * l2.laplacian_ops;
  std::vector<Eigen::Triplet<float>> lap_triplets;
  append_triplets_from_vectorized_sparse(h_weighted_laplacian, 0, 0, 1.0f,
                                         lap_triplets);
  Eigen::SparseMatrix<float> h_laplacian_ops{state_num, state_num};
  h_laplacian_ops.setFromTriplets(lap_triplets.begin(), lap_triplets.end());

  // Assemble jacobian ops.
  std::vector<float> h_jacobian_ops(l2.jacobian_ops.size() * 54);
  for (int i = 0; i < l2.jacobian_ops.size(); ++i) {
    memcpy(h_jacobian_ops.data() + 54 * i, l2.jacobian_ops[i].data(),
           54 * sizeof(float));
  }

  // Assemble area_sqrt.
  Eigen::VectorXf h_area_sqrt = l2_cache.area.array().sqrt();

  // Assemble mass.
  std::vector<float> h_mass(state_num);
  for (int i = 0; i < l2.mass.size(); ++i) {
    float val = c.density * l2.mass(i);
    h_mass[3 * i] = val;
    h_mass[3 * i + 1] = val;
    h_mass[3 * i + 2] = val;
  }

  this->dt = dt;
  this->penalty = PENALTY;
  this->state_num = state_num;
  this->vert_num = l2_cache.mass.size();
  this->face_num = l2_cache.area.size();
  this->elastic_stiffness = c.elastic_stiffness;
  this->bending_stiffness = c.bending_stiffness;
  this->faces = host_eigen_to_device(l2_cache.F, rt);
  this->weighted_laplacian_ops = BSRMatrix{h_laplacian_ops, 3, {}, rt};
  this->C0 = host_eigen_to_device(
      (c.bending_stiffness * l2.C0).reshaped<Eigen::RowMajor>(), rt);
  this->jacobian_ops = vec_like_to_device<float>(h_jacobian_ops, rt);
  this->area_sqrt = host_eigen_to_device(h_area_sqrt, rt);
  this->mass = vec_like_to_device<float>(h_mass, rt);
  this->weighted_AA = BSRMatrix{h_AA, 3, {}, rt};
  this->part_offsets =
      vec_like_to_device<int>(partition.h_partition_offsets, rt);

  rt.stream.sync();
}

}  // namespace silk::cuda
