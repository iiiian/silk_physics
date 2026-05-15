#pragma once

#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"

namespace silk::cuda::solver {

EqualityConstraints gather_barrier_constraints(
    int state_num,
    ctd::span<const ::silk::cuda::collision::Collision> collisions,
    CudaRuntime rt);

}  // namespace silk::cuda::solver
