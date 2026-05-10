#pragma once

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
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
  int state_num = 0;
  Buf<float> mass;
  Buf<float> area;
  Buf<float> jacobian_ops;
  Buf<float> C0;
  BSRMatrix SS_sum;

 public:
  ClothAssemblyL1Cache() = default;
  ClothAssemblyL1Cache(const ClothConfig& config,
                       const ClothAssemblyL2Cache& topology, float dt,
                       CudaRuntime rt);
};

}  // namespace silk::cuda
