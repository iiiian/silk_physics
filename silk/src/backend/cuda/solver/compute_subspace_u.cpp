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
  SPDLOG_INFO("Start computing eigen decomposition.");

  const int n = static_cast<int>(H.rows());
  if (H.rows() != H.cols() || r <= 0 || r > n) {
    SPDLOG_ERROR(
        "Fail to compute subspace u. Reason: Invalid matrix size or r.");
    return std::nullopt;
  }

  using Op = Spectra::SparseSymMatProd<float>;
  Op op(H);
  Spectra::SymEigsSolver<Op> eigs(op, 32, 128);
  eigs.init();
  eigs.compute(Spectra::SortRule::LargestAlge);

  if (eigs.info() == Spectra::CompInfo::Successful) {
    SPDLOG_INFO("Finish computing subspace u.");
    return eigs.eigenvectors();
  }

  SPDLOG_ERROR("Fail to compute subspace u. Reason: Eigen solver failed.");
  return std::nullopt;
}

}  // namespace silk::cuda
