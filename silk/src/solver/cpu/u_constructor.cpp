#include <Spectra/SymEigsShiftSolver.h>
#include <Spectra/MatOp/SparseSymShiftSolve.h>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>

/**
 * @brief Construct Matrix U by combininig r eigenvectors from Matrix H
 *
 * How to use:
 *   bool construct_U(const Eigen::SparseMatrix<double>& H, int r, Eigen::MatrixXf& U_out,
 *                                    int max_subspace=-1, int max_iter=1000, double tol=1e-6);
 */
bool construct_U(
    //Matrix H
    const Eigen::SparseMatrix<double>& H,
    //num of subspace eigenvector
    int r,
    // n * r output
    Eigen::MatrixXf& U_out,
    int max_subspace,
    int max_iter,
    double tol
    )
{   
    const int n = static_cast<int>(H.rows());
    //check if it is a square and correctness of r
    if (H.rows() != H.cols() || r <= 0 || r > n) {
        std::cerr << "[u_constructor] invalid size or r.\n";
        return false;
    }

    // turn it to (H−σI)^-1 in order to calculate eigenvectors
    const double sigma = 0.0;
    Spectra::SparseSymShiftSolve<double> op(H);

    //workspace to find actual subspace
    // M     
    int m = (max_subspace > 0) ? max_subspace : std::min(n, std::max(2 * r + 8, r + 8));

    // Initialize Solver
    // Inputs: goal(smallest eigenvector),r, m, shift(now has set to 0 )
    Spectra::SymEigsShiftSolver<double, Spectra::SMALLEST_MAGN, Spectra::SparseSymShiftSolve<double>> eigs(&op, r, m, sigma);
    eigs.init();

    // Computation begin 
    const int nconv = eigs.compute(max_iter, tol, Spectra::SMALLEST_MAGN);
    if (eigs.info() != Spectra::SUCCESS || nconv < r) {
        std::cerr << "[u_constructor] eigen decomposition failed. converged=" << nconv << "\n";
        return false;
    }

    // take r smallest eigenvector
    Eigen::MatrixXd V = eigs.eigenvectors();       // n x r

    // QRorthog
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(V);
    Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(n, r);

    //output
    U_out = Q.cast<float>();
    return true;
}

