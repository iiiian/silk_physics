#include <backend/cuda/csr_matrix.hpp>
#include <backend/cuda/cuda_utils.hpp>

namespace silk::cuda {

void inexact_solve(int n, int r, CSRMatrix& d_R, const float* d_D,
                   const float* d_rhs, const float* d_U, const float* d_x0,
                   float* d_x) {}

}  // namespace silk::cuda
