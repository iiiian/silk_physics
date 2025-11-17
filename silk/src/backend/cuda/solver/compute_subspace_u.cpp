#include "backend/cuda/solver/compute_subspace_u.hpp"

#include <Spectra/MatOp/SparseSymShiftSolve.h>
#include <Spectra/SymEigsShiftSolver.h>
#include <Spectra/Util/SelectionRule.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <optional>

#include "common/logger.hpp"

std::optional<Eigen::MatrixXf> compute_subspace_u(
    const Eigen::SparseMatrix<float>& H, int r, int max_subspace, int max_iter,
    float tol);
const int n = static_cast<int>(H.rows());
if (H.rows() != H.cols() || r <= 0 || r > n) {
  SPDLOG_ERROR("Fail to compute subspace u. Reason: Invalid matrix size or r.");
  return std::nullopt;
}

// Shift-invert at sigma = 0, target smallest |lambda| of H
const float sigma = 0.0;
Spectra::SparseSymShiftSolve<float> op(H);

// !! Check 1: working subspace size m
// m = r + p (p ≈ 10–40), at least 2r + 8
int m =
    (max_subspace > 0) ? max_subspace : std::min(n, std::max(2 * r + 8, r + 8));

using Op = Spectra::SparseSymShiftSolve<float>;
using Solver = Spectra::SymEigsShiftSolver<Op>;
using SortRule = Spectra::SortRule;

Solver eigs(op, r, m, sigma);
eigs.init();

const int nconv = static_cast<int>(
    eigs.compute(SortRule::LargestMagn, max_iter, tol, SortRule::LargestMagn));

const int info_code = static_cast<int>(eigs.info());
if (info_code != 0 || nconv < r) {
  SPDLOG_ERROR(
      "Fail to compute subspace u. Reason: Eigen solver failed. info code {}",
      info_code);
  return std::nullopt;
}

return eigs.eigenvectors();  // n x r
}
