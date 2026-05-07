#pragma once

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "common/cloth_topology.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

/// Dynamic, time step or config dependent quantities used by the cloth solver.
///
/// Notation:
/// state_num = 3 * vertex num.
class ClothSolverContext {
 public:
  float dt;
  int state_num = 0;
  Buf<float> mass;
  Buf<float> area;
  Buf<float> jacobian_ops;
  Buf<float> C0;
  BSRMatrix H;

 public:
  ClothSolverContext() = default;
  ClothSolverContext(const ClothConfig& config, const ClothTopology& topology,
                     float dt, CudaRuntime rt);
};

}  // namespace silk::cuda
