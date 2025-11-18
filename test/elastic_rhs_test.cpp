#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <catch2/catch_test_macros.hpp>
#include <unsupported/Eigen/KroneckerProduct>

#include <cuda_runtime_api.h>

#include "backend/cuda/solver/cloth_solver_kernel.hpp"

using Eigen::Matrix3f;
using Eigen::Vector3f;

namespace {

// Minimal copy of triangle_jacobian_operator from cloth_topology.cpp.
Eigen::Matrix<float, 6, 9> triangle_jacobian_operator(
    const Vector3f& v0, const Vector3f& v1, const Vector3f& v2) {
  // Convert triangle to 2D.
  Vector3f e0 = v1 - v0;
  Vector3f e1 = v2 - v0;

  Vector3f e0xe1 = e0.cross(e1);
  float area2_eps =
      std::pow(1e-6f * std::max(e0.squaredNorm(), e1.squaredNorm()), 2);
  if (e0xe1.squaredNorm() < area2_eps) {
    // Degenerate triangle; keep behavior but skip logging here.
  }

  // Define 2D basis x and y.
  Vector3f bx = e0.normalized();
  Vector3f by = e0xe1.cross(e0).normalized();

  // The matrix dX is the displacement of the initial triangle in a 2D basis.
  // Definition: dX = ( d1 d2 ).
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e0);
  dX(1, 0) = 0.0f;
  dX(0, 1) = bx.dot(e1);
  dX(1, 1) = by.dot(e1);

  // D is the displacement operator in 3D Cartesian basis.
  // The matrix dx equals ( d1 d2 ) = x * D where x = ( v1 v2 v3 ).
  const Eigen::Matrix<float, 3, 2> D =
      (Eigen::Matrix<float, 3, 2>() << -1, -1, 1, 0, 0, 1).finished();

  // Deformation: F = dx * (dX)^-1 = x * D * (dX)^-1.
  // We vectorize via the Kronecker identity (B^T ⊗ I3) vec(x) = vec(AXB).
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

Eigen::Matrix<float, 9, 1> compute_cpu_elastic_rhs(
    float elastic_stiffness, const Matrix3f& V,
    const Eigen::Matrix<float, 9, 1>& state) {
  // Single-triangle jacobian operator and area.
  Eigen::Matrix<float, 6, 9> jac_op =
      triangle_jacobian_operator(V.row(0), V.row(1), V.row(2));

  Eigen::Matrix<float, 6, 1> Dvec = jac_op * state;
  Eigen::Matrix<float, 3, 2> D = Eigen::Map<Eigen::Matrix<float, 3, 2>>(
      Dvec.data(), 3, 2);  // column-major 3x2

  // SVD and clamping exactly as in CPU cloth_solver_utils.
  Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
      D, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Vector2f sigma = svd.singularValues();
  sigma(0) = std::clamp(sigma(0), 0.9f, 1.1f);
  sigma(1) = std::clamp(sigma(1), 0.9f, 1.1f);

  Eigen::Matrix<float, 3, 2> T = svd.matrixU().block<3, 2>(0, 0) *
                                 sigma.asDiagonal() *
                                 svd.matrixV().transpose();

  Eigen::Matrix<float, 6, 1> Tvec =
      Eigen::Map<Eigen::Matrix<float, 6, 1>>(T.data(), 6, 1);

  // Per-face area (matches igl::doublearea / 2).
  Vector3f e0 = V.row(1) - V.row(0);
  Vector3f e1 = V.row(2) - V.row(0);
  float area = 0.5f * e0.cross(e1).norm();
  float weight = elastic_stiffness * area;

  Eigen::Matrix<float, 9, 1> rhs =
      weight * jac_op.transpose() * Tvec;  // local 9x1 contribution
  return rhs;
}

}  // namespace

TEST_CASE("elastic_rhs_cuda_matches_cpu_single_triangle", "[elastic][cuda]") {
  // Simple non-degenerate triangle.
  Matrix3f V;
  V << 0.0f, 0.0f, 0.0f,  //
       1.0f, 0.0f, 0.0f,  //
       0.2f, 0.9f, 0.3f;

  // Pack vertex positions into a 9x1 state vector (v0, v1, v2).
  Eigen::Matrix<float, 9, 1> state;
  state.segment<3>(0) = V.row(0).transpose();
  state.segment<3>(3) = V.row(1).transpose();
  state.segment<3>(6) = V.row(2).transpose();

  float elastic_stiffness = 2.5f;

  // CPU reference.
  Eigen::Matrix<float, 9, 1> cpu_rhs =
      compute_cpu_elastic_rhs(elastic_stiffness, V, state);

  // GPU elastic RHS for a single face.
  int h_F[3] = {0, 1, 2};

  Eigen::Matrix<float, 6, 9> jac_op =
      triangle_jacobian_operator(V.row(0), V.row(1), V.row(2));
  float h_jac[54];
  static_assert(sizeof(h_jac) == sizeof(float) * 6 * 9,
                "jacobian buffer size mismatch");
  std::memcpy(h_jac, jac_op.data(), sizeof(h_jac));

  Vector3f e0 = V.row(1) - V.row(0);
  Vector3f e1 = V.row(2) - V.row(0);
  float h_area[1] = {0.5f * e0.cross(e1).norm()};

  float h_state[9];
  std::memcpy(h_state, state.data(), sizeof(h_state));

  int* d_F = nullptr;
  float *d_jac = nullptr, *d_area = nullptr, *d_state = nullptr, *d_rhs = nullptr;

  REQUIRE(cudaMalloc((void**)&d_F, 3 * sizeof(int)) == cudaSuccess);
  REQUIRE(cudaMalloc((void**)&d_jac, 54 * sizeof(float)) == cudaSuccess);
  REQUIRE(cudaMalloc((void**)&d_area, sizeof(float)) == cudaSuccess);
  REQUIRE(cudaMalloc((void**)&d_state, 9 * sizeof(float)) == cudaSuccess);
  REQUIRE(cudaMalloc((void**)&d_rhs, 9 * sizeof(float)) == cudaSuccess);

  REQUIRE(cudaMemcpy(d_F, h_F, 3 * sizeof(int), cudaMemcpyHostToDevice) ==
          cudaSuccess);
  REQUIRE(cudaMemcpy(d_jac, h_jac, 54 * sizeof(float),
                     cudaMemcpyHostToDevice) == cudaSuccess);
  REQUIRE(cudaMemcpy(d_area, h_area, sizeof(float),
                     cudaMemcpyHostToDevice) == cudaSuccess);
  REQUIRE(cudaMemcpy(d_state, h_state, 9 * sizeof(float),
                     cudaMemcpyHostToDevice) == cudaSuccess);
  REQUIRE(cudaMemset(d_rhs, 0, 9 * sizeof(float)) == cudaSuccess);

  silk::cuda::compute_elastic_rhs(1, elastic_stiffness, d_F, d_state, d_jac,
                                  d_area, d_rhs);
  REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
  REQUIRE(cudaGetLastError() == cudaSuccess);

  float h_rhs_gpu[9];
  REQUIRE(cudaMemcpy(h_rhs_gpu, d_rhs, 9 * sizeof(float),
                     cudaMemcpyDeviceToHost) == cudaSuccess);

  REQUIRE(cudaFree(d_F) == cudaSuccess);
  REQUIRE(cudaFree(d_jac) == cudaSuccess);
  REQUIRE(cudaFree(d_area) == cudaSuccess);
  REQUIRE(cudaFree(d_state) == cudaSuccess);
  REQUIRE(cudaFree(d_rhs) == cudaSuccess);

  Eigen::Matrix<float, 9, 1> gpu_rhs;
  std::memcpy(gpu_rhs.data(), h_rhs_gpu, sizeof(h_rhs_gpu));

  // Allow a small numerical tolerance due to different SVD implementations.
  REQUIRE(cpu_rhs.size() == gpu_rhs.size());
  INFO("cpu_rhs = " << cpu_rhs.transpose());
  INFO("gpu_rhs = " << gpu_rhs.transpose());
  CHECK(cpu_rhs.isApprox(gpu_rhs, 1e-4f));
}

