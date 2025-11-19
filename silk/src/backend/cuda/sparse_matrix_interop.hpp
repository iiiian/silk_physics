#pragma once

#include <Eigen/SparseCore>

#include "backend/cuda/csr_matrix.hpp"

namespace silk::cuda {

CSRMatrix make_csr_from_eigen(
    const Eigen::SparseMatrix<float, Eigen::RowMajor>& m);

}
