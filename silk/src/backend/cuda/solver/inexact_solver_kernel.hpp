#pragma once

namespace silk::cuda {

void compute_subspace_d32_rhs(int n,
                              const float* d_U,    // n x 32, column-major
                              const float* d_HX,   // n
                              const float* d_rhs,  // n
                              float* d_srhs        // 32
);

void project_subspace_d32_sol(int n, const float* d_U, const float* d_X,
                              const float* ssol, float* out);

}  // namespace silk::cuda
