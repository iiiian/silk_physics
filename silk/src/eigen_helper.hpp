#pragma once

#include <Eigen/SparseCore>
#include <vector>

namespace silk {

template <typename Scalar>
void sparse_to_triplets(const Eigen::SparseMatrix<Scalar>& m,
                        std::vector<Eigen::Triplet<Scalar>>& triplets,
                        int row_offset = 0, int col_offset = 0) {
  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {  // it++ doesn't work
      triplets.emplace_back(row_offset + it.row(), col_offset + it.col(),
                            it.value());
    }
  }
}

template <typename Scalar>
void vectorize_sparse_to_triplets(const Eigen::SparseMatrix<Scalar>& m,
                                  std::vector<Eigen::Triplet<Scalar>>& triplets,
                                  int row_offset = 0, int col_offset = 0) {
  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {
      int base_row = row_offset + 3 * it.row();
      int base_col = col_offset + 3 * it.col();
      triplets.emplace_back(base_row, base_col, it.value());
      triplets.emplace_back(base_row + 1, base_col + 1, it.value());
      triplets.emplace_back(base_row + 2, base_col + 2, it.value());
    }
  }
}

}  // namespace silk
