#include "backend/cuda/csr_matrix.hpp"

#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

#include "backend/cuda/copy_vector_like.hpp"
#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

CSRMatrix::CSRMatrix(cusparseSpMatDescr_t desc) {
  int64_t rows, cols, nnz;
  void *row_offsets, *colind, *values;
  cusparseIndexType_t row_offset_type, colind_type;
  cusparseIndexBase_t idx_base;
  cudaDataType value_type;
  CHECK_CUSPARSE(cusparseCsrGet(desc, &rows, &cols, &nnz, &row_offsets, &colind,
                                &values, &row_offset_type, &colind_type,
                                &idx_base, &value_type));
  assert(row_offset_type == CUSPARSE_INDEX_32I);
  assert(colind_type == CUSPARSE_INDEX_32I);
  assert(idx_base == CUSPARSE_INDEX_BASE_ZERO);
  assert(value_type == CUDA_R_32F);

  row_num = static_cast<int>(rows);
  col_num = static_cast<int>(cols);
  non_zero_num = static_cast<int>(nnz);
  d_row_ptr = static_cast<int*>(row_offsets);
  d_col_idx = static_cast<int*>(colind);
  d_values = static_cast<float*>(values);
}

CSRMatrix::CSRMatrix(CSRMatrix&& other) noexcept { swap(other); }

CSRMatrix& CSRMatrix::operator=(CSRMatrix&& other) noexcept {
  swap(other);
  return *this;
}

CSRMatrix::~CSRMatrix() {
  CHECK_CUDA(cudaFree(d_row_ptr));
  CHECK_CUDA(cudaFree(d_col_idx));
  CHECK_CUDA(cudaFree(d_values));
}

CSRMatrixView CSRMatrix::get_view() const {
  CSRMatrixView v;
  v.row_num = row_num;
  v.col_num = col_num;
  v.non_zero_num = non_zero_num;
  v.d_row_ptr = d_row_ptr;
  v.d_col_idx = d_col_idx;
  v.d_values = d_values;

  return v;
}

cusparseSpMatDescr_t CSRMatrix::get_cusparse_desc() {
  cusparseSpMatDescr_t desc;
  CHECK_CUSPARSE(cusparseCreateCsr(
      &desc, static_cast<int64_t>(row_num), static_cast<int64_t>(col_num),
      static_cast<int64_t>(non_zero_num), static_cast<void*>(d_row_ptr),
      static_cast<void*>(d_col_idx), static_cast<void*>(d_values),
      CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO,
      CUDA_R_32F));
  return desc;
}

cusparseConstSpMatDescr_t CSRMatrix::get_const_cusparse_desc() const {
  cusparseConstSpMatDescr_t desc;
  CHECK_CUSPARSE(cusparseCreateConstCsr(
      &desc, static_cast<int64_t>(row_num), static_cast<int64_t>(col_num),
      static_cast<int64_t>(non_zero_num), static_cast<const void*>(d_row_ptr),
      static_cast<const void*>(d_col_idx), static_cast<const void*>(d_values),
      CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO,
      CUDA_R_32F));
  return desc;
}

void CSRMatrix::swap(CSRMatrix& other) noexcept {
  if (this == &other) {
    return;
  }
  std::swap(row_num, other.row_num);
  std::swap(col_num, other.col_num);
  std::swap(non_zero_num, other.non_zero_num);
  std::swap(d_row_ptr, other.d_row_ptr);
  std::swap(d_col_idx, other.d_col_idx);
  std::swap(d_values, other.d_values);
}

}  // namespace silk::cuda
