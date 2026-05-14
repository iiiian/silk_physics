#pragma once

#include <cstdint>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"

namespace silk::cuda::assembly {

void assemble_cloth(ObjRegistry& registry, uint32_t entity, float dt,
                    int state_offset, CudaRuntime rt);

// void batch_compute_cloth_invariant_rhs(ObjRegistry& registry, float* d_rhs);

// bool batch_compute_cloth_outer_loop(ObjRegistry& registry, const float*
// d_state,
//                                     const float* d_state_velocity,
//                                     const BarrierConstrain&
//                                     barrier_constrain, const Eigen::Vector3f&
//                                     state_acceleration, float* d_rhs);

// bool batch_compute_cloth_inner_loop(ObjRegistry& registry,
//                                     const float* d_outer_rhs,
//                                     const BarrierConstrain&
//                                     barrier_constrain, float* d_state);

}  // namespace silk::cuda::assembly
