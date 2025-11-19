#include <Spectra/SymEigsShiftSolver.h>
#include <Spectra/MatOp/SparseSymShiftSolve.h>
#include <Spectra/Util/SelectionRule.h>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>

/**
 * @brief Construct matrix U by combining r eigenvectors of H (smallest |lambda| of H)
 * 
 * @param H Symmetric positive definite sparse matrix.
 * @param r Number of eigenvectors to compute.
 * @param U_out Output matrix containing orthogonal basis (n x r).
 * @param max_subspace Optional maximum subspace dimension.
 * @param max_iter Maximum number of iterations for eigen solver.
 * @param tol Convergence tolerance.
 * @return true if eigen decomposition succeeded, false otherwise.
 */

// to use:
//  Eigen::MatrixXf U;
//  bool ok = construct_U(H, r, U);

bool construct_u(const Eigen::SparseMatrix<double>& H,
                 int r,
                 Eigen::MatrixXf& U_out,
                 int max_subspace /* = -1 */,
                 int max_iter /* = 1000 */,
                 double tol /* = 1e-6 */) 
{
    const int n = static_cast<int>(H.rows());
    if (H.rows() != H.cols() || r <= 0 || r > n) {
        std::cerr << "[u_constructor] invalid size or r.\n";
        return false;
    }

    // Shift-invert at sigma = 0, target smallest |lambda| of H
    const double sigma = 0.0;
    Spectra::SparseSymShiftSolve<double> op(H);

    // !! Check 1: working subspace size m
    // m = r + p (p ≈ 10–40), at least 2r + 8
    int m = (max_subspace > 0)
              ? max_subspace
              : std::min(n, std::max(2 * r + 8, r + 8));

    using Op = Spectra::SparseSymShiftSolve<double>;
    using Solver = Spectra::SymEigsShiftSolver<Op>;
    using SortRule = Spectra::SortRule;

    Solver eigs(op, r, m, sigma);
    eigs.init();

    const int nconv = static_cast<int>(
        eigs.compute(SortRule::LargestMagn, max_iter, tol, SortRule::LargestMagn));

    const int info_code = static_cast<int>(eigs.info());
    if (info_code != 0 || nconv < r) {
        std::cerr << "[u_constructor] eigensolver failed. info=" << info_code
                  << ", converged=" << nconv << " (< r=" << r << ")\n";
        return false;
    }

    Eigen::MatrixXd V = eigs.eigenvectors();  // n x r

    // !! Check 2: orthogonalize eigenvectors
    Eigen::MatrixXd Q;
    if (false) {
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(V);
        Q = qr.householderQ() * Eigen::MatrixXd::Identity(n, r);
    } else {
        U_out = V.cast<float>();
        return true;
    }

    U_out = Q.cast<float>();
    return true;
}
