#pragma once

namespace silk::cuda {

void compute_subspace_d32_rhs(int n,
                              const float* d_U,        // n x 32, column-major
                              const float* d_HX,       // n
                              const float* d_rhs,      // n
                              const float* d_delta_H,  // n (barrier lhs)
                              const float* d_X,        // n (subspace center)
                              float* d_srhs            // 32
);

// Compute U^T (ΔH) U where ΔH is diagonal with entries d_delta_H.
// The result is a 32x32 column-major matrix stored in d_UDHU.
void compute_subspace_d32_UdHU(int n, const float* d_U, const float* d_dH,
                               float* d_UDHU);

void project_subspace_d32_sol(int n, const float* d_U, const float* d_X,
                              const float* ssol, float* out);

}  // namespace silk::cuda
