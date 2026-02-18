#include "backend/cuda/solver/compute_subspace_u.hpp"

#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/SymEigsSolver.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <optional>

#include "common/logger.hpp"

namespace silk::cuda {

std::optional<Eigen::MatrixXf> compute_subspace_u(
    const Eigen::SparseMatrix<float>& H, int r, int max_subspace, int max_iter,
    float tol) {
  // Disable subspace u for now.
  return Eigen::MatrixXf::Zero(H.rows(), 32);

  SPDLOG_INFO("Start computing eigen decomposition.");

  const int n = static_cast<int>(H.rows());
  if (H.rows() != H.cols() || r <= 0 || r > n) {
    SPDLOG_ERROR(
        "Fail to compute subspace u. Reason: Invalid matrix size or r.");
    return std::nullopt;
  }

  using Op = Spectra::SparseSymMatProd<float>;
  Op op(H);

  int nev = r;
  int ncv = (max_subspace > 0) ? max_subspace
                               : std::min(std::max(2 * r + 1, r + 1), n);
  ncv = std::min(std::max(nev + 1, ncv), n);

  Spectra::SymEigsSolver<Op> eigs(op, nev, ncv);
  eigs.init();
  eigs.compute(Spectra::SortRule::SmallestAlge, max_iter, tol);

  if (eigs.info() == Spectra::CompInfo::Successful) {
    SPDLOG_INFO("Finish computing subspace u.");
    return eigs.eigenvectors();
  }

  SPDLOG_ERROR("Fail to compute subspace u. Reason: Eigen solver failed.");
  return std::nullopt;
}

}  // namespace silk::cuda
