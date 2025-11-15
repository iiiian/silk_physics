#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <iostream>
#include <chrono>
#include <cuda_runtime.h>

#include "silk/src/solver/gpu/jacobi_solver.cuh"

int main() {
    std::cout << "Testing GPU Jacobi Solver with larger matrix..." << std::endl;

    // Create a larger test matrix - 100x100 diagonally dominant
    const int n = 100;
    std::vector<Eigen::Triplet<float>> triplets;

    // Fill diagonal with dominant values
    for (int i = 0; i < n; ++i) {
        triplets.emplace_back(i, i, 4.0f);
    }

    // Add off-diagonal elements
    for (int i = 0; i < n - 1; ++i) {
        triplets.emplace_back(i, i + 1, -1.0f);
        triplets.emplace_back(i + 1, i, -1.0f);
    }

    // Create sparse matrix in row-major format
    Eigen::SparseMatrix<float, Eigen::RowMajor> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();

    std::cout << "Matrix A: " << n << "x" << n << " with " << A.nonZeros() << " non-zeros" << std::endl;

    // Create RHS vector: b = ones
    Eigen::VectorXf b(n);
    b.setOnes();

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
    auto start = std::chrono::high_resolution_clock::now();

    if (!solver.solve(D_host, b, x, 1000, 1e-6f)) {
        std::cout << "GPU Jacobi solve failed!" << std::endl;
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Print first 5 and last 5 elements of solution
    std::cout << "Solution (first 5): ";
    for (int i = 0; i < std::min(5, n); ++i) {
        std::cout << x(i) << " ";
    }
    std::cout << std::endl;

    std::cout << "Solution (last 5): ";
    for (int i = std::max(0, n-5); i < n; ++i) {
        std::cout << x(i) << " ";
    }
    std::cout << std::endl;

    // Verify solution by computing residual
    Eigen::VectorXf residual = A * x - b;
    float residual_norm = residual.norm();

    std::cout << "Residual norm: " << residual_norm << std::endl;
    std::cout << "Solve time: " << duration.count() << " microseconds" << std::endl;

    if (residual_norm < 1e-5f) {
        std::cout << "✓ GPU Jacobi solver large test PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "✗ GPU Jacobi solver large test FAILED!" << std::endl;
        return 1;
    }
}