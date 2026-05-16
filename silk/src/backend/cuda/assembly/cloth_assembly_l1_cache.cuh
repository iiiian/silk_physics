#pragma once

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "common/cloth_assembly_l2_cache.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

/// Dynamic, time step or config dependent quantities used by the cloth solver.
///
/// Notation:
/// state_num = 3 * vertex num.
class ClothAssemblyL1Cache {
 public:
  float dt;
  float penalty;
  int vert_num;
  int face_num;
  int state_num;
  float elastic_stiffness;
  float bending_stiffness;

  Buf<int> faces;
  BSRMatrix weighted_laplacian_ops;
  Buf<float> C0;
  Buf<float> weighted_jacobian_ops;
  Buf<float> mass;
  BSRMatrix weighted_AA;
  Buf<int> part_offsets;

 public:
  ClothAssemblyL1Cache() = default;
  ClothAssemblyL1Cache(const ClothConfig& config,
                       const MeshPartition& partition,
                       const ClothAssemblyL2Cache& l2_cache, float dt,
                       CudaRuntime rt);
};

}  // namespace silk::cuda
