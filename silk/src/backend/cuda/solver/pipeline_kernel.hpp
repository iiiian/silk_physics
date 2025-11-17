#pragma once

#include <backend/cuda/collision/collision.hpp>
#include <backend/cuda/solver/barrier_constrain.hpp>

namespace silk::cuda {

void predict(int state_num, float dt, float acc_x, float acc_y, float acc_z,
             const float* d_curr_state, const float* d_state_velocity,
             float* d_next_state);

float compute_L2_distance(int num, const float* d_a, const float* d_b,
                          float* d_buffer);

void enforce_barrier_constrain(const BarrierConstrain& barrer, float* state);

void vec_mix(int num, float nr, const float* d_a, const float* d_b,
             float* d_out);

}  // namespace silk::cuda
