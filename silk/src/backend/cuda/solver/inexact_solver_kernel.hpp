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

// Compute U^T (ΔH) U where ΔH is diagonal with entries in d_dH_global at
// the indices listed in d_index. U is provided in row-major layout per DOF.
// Only DOFs in [state_offset, state_offset + state_size) are considered.
// The result is a 32x32 column-major matrix stored in d_UDHU.
void compute_subspace_d32_UdHU(int constrain_num, const int* d_index,
                               const float* d_dH_global,
                               const float* d_U_row_major, int state_offset,
                               int state_size, float* d_UDHU);

void project_subspace_d32_sol(int n, const float* d_U, const float* d_X,
                              const float* ssol, float* out);

}  // namespace silk::cuda
