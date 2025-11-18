#pragma once

#include <cusparse.h>

namespace silk::cuda {

struct CSRMatrixView {
  int row_num = 0;
  int col_num = 0;
  int non_zero_num = 0;
  int* d_row_ptr = nullptr;
  int* d_col_idx = nullptr;
  float* d_values = nullptr;
};

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
  CSRMatrix(cusparseSpMatDescr_t desc);
  CSRMatrix(const CSRMatrix& other) = delete;
  CSRMatrix(CSRMatrix&& other) noexcept;
  CSRMatrix& operator=(const CSRMatrix& other) = delete;
  CSRMatrix& operator=(CSRMatrix&& other) noexcept;
  ~CSRMatrix();

  CSRMatrixView get_view() const;
  cusparseSpMatDescr_t get_cusparse_desc();
  cusparseConstSpMatDescr_t get_const_cusparse_desc() const;

 private:
  void swap(CSRMatrix& other) noexcept;
};

}  // namespace silk::cuda
