#pragma once

namespace silk::cuda {

void vector_add(int num, const float* d_a, const float* d_b, float* d_c);

void vector_add(int num, const float* d_a, float b, float* d_c);

void compute_outer_rhs(int state_num, float dt, float acc_x, float acc_y,
                       float acc_z, const float* d_mass, const float* d_state,
                       const float* d_state_velocity,
                       const float* d_barrier_rhs, float* d_rhs);

void compute_elastic_rhs(int face_num, float elastic_stiffness, const int* d_F,
                         const float* d_state, const float* d_jacobian_ops,
                         const float* d_areas, float* d_rhs);

}  // namespace silk::cuda
