/**
 * @file eigen_utils.hpp
 * @brief Utility functions for manipulating Eigen sparse matrices and triplets.
 *
 * Provides helper functions for extracting triplets from sparse matrices with
 * offset transformations and symmetry handling, commonly used in finite element
 * assembly operations.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

#include "symmetry.hpp"

namespace silk {

/**
 * @brief Extracts triplets from a sparse matrix with offset and scaling
 * transformations.
 *
 * @tparam Scalar The scalar type of the matrix elements
 * @param m The source sparse matrix (must be square)
 * @param row_offset Offset added to all row indices
 * @param col_offset Offset added to all column indices
 * @param scaling Factor multiplied with all matrix values
 * @param triplets Output vector where triplets are appended
 * @param symmetry Symmetry filter applied to source matrix indices
 */
template <typename Scalar>
void append_triplets_from_sparse(const Eigen::SparseMatrix<Scalar>& m,
                                 int row_offset, int col_offset, Scalar scaling,
                                 std::vector<Eigen::Triplet<Scalar>>& triplets,
                                 Symmetry symmetry = Symmetry::NotSymmetric) {
  assert((m.rows() == m.cols()));

  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {
      // Skip entries based on symmetry.
      if (symmetry == Symmetry::Lower) {
        if (it.row() < it.col()) {
          continue;
        }
      } else if (symmetry == Symmetry::Upper) {
        if (it.row() > it.col()) {
          continue;
        }
      }

      triplets.emplace_back(row_offset + it.row(), col_offset + it.col(),
                            scaling * it.value());
    }
  }
}

/**
 * @brief Extracts triplets from a sparse matrix with 3x3 block expansion.
 *
 * @tparam Scalar The scalar type of the matrix elements
 * @param m The source sparse matrix
 * @param row_offset Base offset for row indices (before 3x multiplication)
 * @param col_offset Base offset for column indices (before 3x multiplication)
 * @param scaling Factor multiplied with all matrix values
 * @param triplets Output vector where expanded triplets are appended
 * @param symmetry Symmetry filter applied to source matrix indices
 */
template <typename Scalar>
void append_triplets_from_vectorized_sparse(
    const Eigen::SparseMatrix<Scalar>& m, int row_offset, int col_offset,
    Scalar scaling, std::vector<Eigen::Triplet<Scalar>>& triplets,
    Symmetry symmetry = Symmetry::NotSymmetric) {
  using Iter = typename Eigen::SparseMatrix<Scalar>::InnerIterator;
  for (int i = 0; i < m.outerSize(); ++i) {
    for (Iter it(m, i); it; ++it) {
      // Skip entries based on symmetry.
      if (symmetry == Symmetry::Lower) {
        if (it.row() < it.col()) {
          continue;
        }
      } else if (symmetry == Symmetry::Upper) {
        if (it.row() > it.col()) {
          continue;
        }
      }

      // Expand each scalar entry into a 3x3 diagonal block.
      int base_row = row_offset + 3 * it.row();
      int base_col = col_offset + 3 * it.col();
      Scalar val = scaling * it.value();
      triplets.emplace_back(base_row, base_col, val);          // (0,0)
      triplets.emplace_back(base_row + 1, base_col + 1, val);  // (1,1)
      triplets.emplace_back(base_row + 2, base_col + 2, val);  // (2,2)
    }
  }
}

}  // namespace silk
