#pragma once

#include "backend/cuda/csr_matrix.hpp"

namespace silk::cuda {

bool a_jacobi(int n, int max_iter, float abs_tol, float rel_tol,
              const CSRMatrix& d_R, const float* d_D, const float* d_rhs,
              float* d_x);

}
