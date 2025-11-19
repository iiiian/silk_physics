#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <optional>

namespace silk::cuda {

/**
 * @brief Construct matrix U by combining r eigenvectors of H (smallest |lambda|
 * of H)
 *
 * @param H Symmetric positive definite sparse matrix.
 * @param r Number of eigenvectors to compute.
 * @param max_subspace Optional maximum subspace dimension.
 * @param max_iter Maximum number of iterations for eigen solver.
 * @param tol Convergence tolerance.
 * @return Output matrix containing orthogonal basis (n x r). nullopt if failed.
 */
std::optional<Eigen::MatrixXf> compute_subspace_u(
    const Eigen::SparseMatrix<float>& H, int r, int max_subspace = -1,
    int max_iter = 1000, float tol = 1e-6);

}  // namespace silk::cuda
