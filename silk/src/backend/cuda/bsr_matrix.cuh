#pragma once

#include <Eigen/SparseCore>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

struct BSRView {
  int dim;                      //< BSR matrix dim.
  int block_dim;                //< dim of a block.
  int padded_block;             //< block id that is padded.
  int padded_scalar_num;        //< padded scalar num.
  int non_zeros;                //< BSR non-zeros.
  ctd::span<const int> rows;    //< BSR row ptr.
  ctd::span<const int> cols;    //< BSR cols.
  ctd::span<const float> vals;  //< BSR values.
};

struct DynamicBSRView {
  ctd::span<const float> diag;
  BSRView mat;
};

/// @brief Block CSR matrix.
class BSRMatrix {
 public:
  BSRMatrix() = default;

  /// @brief Build from host CSR matrix. Empty permutation implies identity.
  /// Sync stream before return.
  BSRMatrix(const Eigen::SparseMatrix<float> &A, int block_dim,
            ctd::span<const int> permutation, CudaRuntime rt);

  BSRView view() const {
    // clang-format off
    return BSRView{dim_,
                   block_dim_,
                   padded_block_,
                   padded_scalar_num_,
                   non_zeros_,
                   *rows_,
                   *cols_,
                   *vals_};
    // clang-format on
  }

 private:
  int dim_ = 0;
  int block_dim_ = 0;
  int padded_block_ = -1;
  int padded_scalar_num_ = 0;
  int non_zeros_ = 0;
  Buf<int> rows_;
  Buf<int> cols_;
  Buf<float> vals_;
};

}  // namespace silk::cuda
