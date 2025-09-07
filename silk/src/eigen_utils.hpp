#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

#include "symmetry.hpp"

namespace silk {

template <typename Scalar>
void append_triplets_from_sparse(const Eigen::SparseMatrix<Scalar>& m,
                                 int row_offset, int col_offset, Scalar scaling,
                                 std::vector<Eigen::Triplet<Scalar>>& triplets,
                                 Symmetry sym_stat = Symmetry::NotSymmetric) {
  assert((m.rows() == m.cols()));

  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {  // it++ doesn't work
      if (sym_stat == Symmetry::Lower) {
        if (it.row() < it.col()) {
          continue;
        }
      } else if (sym_stat == Symmetry::Upper) {
        if (it.row() > it.col()) {
          continue;
        }
      }

      triplets.emplace_back(row_offset + it.row(), col_offset + it.col(),
                            scaling * it.value());
    }
  }
}

template <typename Scalar>
void append_triplets_from_vectorized_sparse(
    const Eigen::SparseMatrix<Scalar>& m, int row_offset, int col_offset,
    Scalar scaling, std::vector<Eigen::Triplet<Scalar>>& triplets,
    Symmetry sym_stat = Symmetry::NotSymmetric) {
  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {
      if (sym_stat == Symmetry::Lower) {
        if (it.row() < it.col()) {
          continue;
        }
      } else if (sym_stat == Symmetry::Upper) {
        if (it.row() > it.col()) {
          continue;
        }
      }

      int base_row = row_offset + 3 * it.row();
      int base_col = col_offset + 3 * it.col();
      Scalar val = scaling * it.value();
      triplets.emplace_back(base_row, base_col, val);
      triplets.emplace_back(base_row + 1, base_col + 1, val);
      triplets.emplace_back(base_row + 2, base_col + 2, val);
    }
  }
}

}  // namespace silk
