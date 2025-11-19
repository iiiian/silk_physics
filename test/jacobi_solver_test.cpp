#include <cuda_runtime_api.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <catch2/catch_test_macros.hpp>
#include <random>
#include <vector>

#include "backend/cuda/csr_matrix.hpp"
#include "backend/cuda/solver/a_jacobi_solver.hpp"
#include "backend/cuda/sparse_matrix_interop.hpp"

TEST_CASE("a_jacobi_solve_matches_eigen_ldlt_100x100", "[jacobi][cuda]") {
  constexpr int n = 100;

  std::mt19937 rng(1234);

  // Construct a simple SPD, strictly diagonally dominant tridiagonal system:
  //  A has 10 on the diagonal and -1 on the first off-diagonals so that
  //  diagonal entries are 10x larger than off-diagonals.
  Eigen::MatrixXf A_dense = Eigen::MatrixXf::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    A_dense(i, i) = 10.0f;
    if (i + 1 < n) {
      A_dense(i, i + 1) = -1.0f;
      A_dense(i + 1, i) = -1.0f;
    }
  }
  Eigen::VectorXf b = Eigen::VectorXf::Random(n);

  // CPU reference solution using Eigen LDLT on dense A.
  Eigen::LDLT<Eigen::MatrixXf> ldlt(A_dense);
  REQUIRE(ldlt.info() == Eigen::Success);
  Eigen::VectorXf x_ref = ldlt.solve(b);
  REQUIRE(ldlt.info() == Eigen::Success);

  // Build CUDA CSR matrix for the off-diagonal part R and Jacobi diagonals D
  // consistent with ClothSolverContext: H = diag(D) + R.
  Eigen::SparseMatrix<float, Eigen::RowMajor> R(n, n);
  for (int i = 0; i < n - 1; ++i) {
    R.coeffRef(i, i + 1) = -1.0f;
    R.coeffRef(i + 1, i) = -1.0f;
  }
  R.makeCompressed();
  silk::cuda::CSRMatrix d_R = silk::cuda::make_csr_from_eigen(R);
  Eigen::VectorXf D = Eigen::VectorXf::Constant(n, 10.0f);

  float *d_D = nullptr, *d_b = nullptr, *d_x = nullptr;
  REQUIRE(cudaMalloc((void **)&d_D, n * sizeof(float)) == cudaSuccess);
  REQUIRE(cudaMalloc((void **)&d_b, n * sizeof(float)) == cudaSuccess);
  REQUIRE(cudaMalloc((void **)&d_x, n * sizeof(float)) == cudaSuccess);

  REQUIRE(cudaMemcpy(d_D, D.data(), n * sizeof(float),
                     cudaMemcpyHostToDevice) == cudaSuccess);
  REQUIRE(cudaMemcpy(d_b, b.data(), n * sizeof(float),
                     cudaMemcpyHostToDevice) == cudaSuccess);
  REQUIRE(cudaMemset(d_x, 0, n * sizeof(float)) == cudaSuccess);

  bool ok = silk::cuda::a_jacobi(n, 50000, 1e-6f, 1e-5f, d_R, d_D, d_b, d_x);
  REQUIRE(ok);

  std::vector<float> h_x(n);
  REQUIRE(cudaMemcpy(h_x.data(), d_x, n * sizeof(float),
                     cudaMemcpyDeviceToHost) == cudaSuccess);

  REQUIRE(cudaFree(d_D) == cudaSuccess);
  REQUIRE(cudaFree(d_b) == cudaSuccess);
  REQUIRE(cudaFree(d_x) == cudaSuccess);

  Eigen::VectorXf x_gpu(n);
  for (int i = 0; i < n; ++i) {
    x_gpu(i) = h_x[i];
  }

  // Compare solutions up to a reasonable tolerance for an iterative solver.
  INFO("||x_ref - x_gpu|| = " << (x_ref - x_gpu).norm());
  CHECK(x_ref.isApprox(x_gpu, 1e-3f));
}
