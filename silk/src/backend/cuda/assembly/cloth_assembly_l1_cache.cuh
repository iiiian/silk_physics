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
  int face_num;
  int state_num;
  float elastic_stiffness;

  Buf<int> faces;
  Buf<float> jacobian_ops;
  Buf<float> area_sqrt;
  Buf<float> mass;
  // k C^T W C x_rest, the constant bending contribution to the main RHS.
  Buf<float> bending_rhs;
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
