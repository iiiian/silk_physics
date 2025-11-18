#include "backend/cuda/sparse_matrix_interop.hpp"

#include <Eigen/SparseCore>
#include <cassert>
#include <limits>

#include "backend/cuda/copy_vector_like.hpp"

namespace silk::cuda {

CSRMatrix make_csr_from_eigen(
    const Eigen::SparseMatrix<float, Eigen::RowMajor>& m) {
  assert(m.isCompressed());
  constexpr int MAX_IDX = std::numeric_limits<int>::max();
  assert(m.rows() < MAX_IDX && m.cols() < MAX_IDX && m.nonZeros() < MAX_IDX);

  int row_num = m.rows();
  int col_num = m.cols();
  int non_zero_num = m.nonZeros();

  // In RowMajor Eigen::SparseMatrix, outerIndexPtr() corresponds to row_ptr
  // and innerIndexPtr() corresponds to col indices.
  std::vector<int> h_row_ptr(m.outerSize() + 1);
  for (int i = 0; i < m.outerSize() + 1; ++i) {
    h_row_ptr[i] = m.outerIndexPtr()[i];
  }
  int* d_row_ptr = host_vector_to_device(h_row_ptr);

  std::vector<int> h_col_idx(non_zero_num);
  for (int i = 0; i < non_zero_num; ++i) {
    h_col_idx[i] = m.innerIndexPtr()[i];
  }
  int* d_col_idx = host_vector_to_device(h_col_idx);

  std::vector<float> h_values(non_zero_num);
  for (int i = 0; i < non_zero_num; ++i) {
    h_values[i] = m.valuePtr()[i];
  }
  float* d_values = host_vector_to_device(h_values);

  CSRMatrix csr;
  csr.row_num = row_num;
  csr.col_num = col_num;
  csr.non_zero_num = non_zero_num;
  csr.d_row_ptr = d_row_ptr;
  csr.d_col_idx = d_col_idx;
  csr.d_values = d_values;

  return csr;
}

}  // namespace silk::cuda
