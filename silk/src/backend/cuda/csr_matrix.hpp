#pragma once

#include <cusparse.h>

#include <Eigen/SparseCore>

namespace silk::cuda {

class CSRMatrix {
 public:
  int row_num = 0;
  int col_num = 0;
  int non_zero_num = 0;
  int* d_row_ptr = nullptr;
  int* d_col_idx = nullptr;
  float* d_values = nullptr;

 public:
  CSRMatrix() = default;
  CSRMatrix(const Eigen::SparseMatrix<float>& m);
  CSRMatrix(cusparseSpMatDescr_t desc);
  CSRMatrix(const CSRMatrix& other) = delete;
  CSRMatrix(CSRMatrix&& other) noexcept;
  CSRMatrix& operator=(const CSRMatrix& other) = delete;
  CSRMatrix& operator=(CSRMatrix&& other) noexcept;
  ~CSRMatrix();

  cusparseSpMatDescr_t get_cusparse_desc();
  cusparseConstSpMatDescr_t get_const_cusparse_desc() const;

 private:
  void swap(CSRMatrix& other) noexcept;
};

}  // namespace silk::cuda
