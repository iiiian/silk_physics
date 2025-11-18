#pragma once

#include <Eigen/Core>

#include "backend/cuda/ecs.hpp"
#include "backend/cuda/solver/barrier_constrain.hpp"

namespace silk::cuda {

void batch_reset_cloth_simulation(Registry& registry);

bool prepare_cloth_simulation(Registry& registry, Entity& entity, float dt,
                              int state_offset);

void batch_compute_cloth_invariant_rhs(Registry& registry, float* d_rhs);

bool batch_compute_cloth_outer_loop(Registry& registry, const float* d_state,
                                    const float* d_state_velocity,
                                    const BarrierConstrain& barrier_constrain,
                                    const Eigen::Vector3f& state_acceleration,
                                    float* d_rhs);

bool batch_compute_cloth_inner_loop(Registry& registry,
                                    const float* d_outer_rhs,
                                    const BarrierConstrain& barrier_constrain,
                                    float* d_state);

}  // namespace silk::cuda
