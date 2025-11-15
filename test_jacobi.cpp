#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <iostream>

// Mock GPU Jacobi solver for testing (simplified version)
#include "silk/src/solver/gpu/jacobi_solver.cuh"

int main() {
    std::cout << "Testing GPU Jacobi Solver Integration" << std::endl;

    // Create a simple test matrix - 3x3 diagonally dominant
    const int n = 3;
    std::vector<Eigen::Triplet<float>> triplets;

    // Fill diagonal with dominant values
    triplets.emplace_back(0, 0, 4.0f);
    triplets.emplace_back(1, 1, 4.0f);
    triplets.emplace_back(2, 2, 4.0f);

    // Add off-diagonal elements
    triplets.emplace_back(0, 1, -1.0f);
    triplets.emplace_back(1, 0, -1.0f);
    triplets.emplace_back(1, 2, -1.0f);
    triplets.emplace_back(2, 1, -1.0f);

    // Create sparse matrix in row-major format
    Eigen::SparseMatrix<float, Eigen::RowMajor> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();

    // Create RHS vector: b = [1, 2, 3]
    Eigen::VectorXf b(n);
    b << 1.0f, 2.0f, 3.0f;

    // Solution vector
    Eigen::VectorXf x(n);

    // Test GPU Jacobi solver
    silk::gpu::GpuJacobiSolver solver;
    Eigen::VectorXf D_host(n);

    std::cout << "Setting up GPU Jacobi solver..." << std::endl;
    if (!solver.setup(A, D_host)) {
        std::cout << "GPU Jacobi solver setup failed!" << std::endl;
        return 1;
    }

    std::cout << "Solving system Ax = b using GPU Jacobi..." << std::endl;
    if (!solver.solve(D_host, b, x, 100, 1e-6f)) {
        std::cout << "GPU Jacobi solve failed!" << std::endl;
        return 1;
    }

    std::cout << "Solution: ";
    for (int i = 0; i < n; ++i) {
        std::cout << x(i) << " ";
    }
    std::cout << std::endl;

    // Verify solution by computing residual
    Eigen::VectorXf residual = A * x - b;
    float residual_norm = residual.norm();

    std::cout << "Residual norm: " << residual_norm << std::endl;

    if (residual_norm < 1e-5f) {
        std::cout << "✓ GPU Jacobi solver test PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "✗ GPU Jacobi solver test FAILED!" << std::endl;
        return 1;
    }
}