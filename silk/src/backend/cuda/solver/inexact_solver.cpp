#include "backend/cuda/solver/inexact_solver.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/device_vector.hpp"
#include "backend/cuda/solver/cloth_solver_context.hpp"
#include "backend/cuda/solver/inexact_solver_kernel.hpp"

namespace silk::cuda {

void inexact_solve(const ClothSolverContext& solver_context, const float* d_rhs,
                   const BarrierConstrain& barrier_constrain, int state_offset,
                   float* d_x) {
  auto& c = solver_context;

  DVector<float> d_srhs{32};
  DVector<float> d_UdHU{32 * 32};
  CHECK_CUDA(cudaMemset(d_srhs, 0, 32 * sizeof(float)));
  CHECK_CUDA(cudaMemset(d_UdHU, 0, 32 * 32 * sizeof(float)));
  CHECK_CUDA(cudaDeviceSynchronize());

  // Compute U^T (b - H X - dH X).
  const float* d_barrier_lhs_local = barrier_constrain.d_lhs + state_offset;
  compute_subspace_d32_rhs(c.state_num, c.d_U, c.d_HX, d_rhs,
                           d_barrier_lhs_local, c.d_X, d_srhs);

  // Compute U^T (dH) U using only active barrier entries.
  compute_subspace_d32_UdHU(barrier_constrain.constrain_num,
                            barrier_constrain.d_index, barrier_constrain.d_lhs,
                            c.d_U_RM, state_offset, c.state_num, d_UdHU);
  CHECK_CUDA(cudaDeviceSynchronize());

  Eigen::Vector<float, 32> h_srhs;
  CHECK_CUDA(cudaMemcpy(h_srhs.data(), d_srhs, 32 * sizeof(float),
                        cudaMemcpyDeviceToHost));

  Eigen::Matrix<float, 32, 32> h_UdHU;
  CHECK_CUDA(cudaMemcpy(h_UdHU.data(), d_UdHU, 32 * 32 * sizeof(float),
                        cudaMemcpyDeviceToHost));
  // Eq. (18): (U^T(H + dH)U) q = U^T(b - H X - dH X).
  Eigen::Matrix<float, 32, 32> H_sub = c.UHU + h_UdHU;
  Eigen::Vector<float, 32> h_ssol = H_sub.ldlt().solve(h_srhs);

  CHECK_CUDA(cudaMemcpy(d_srhs, h_ssol.data(), 32 * sizeof(float),
                        cudaMemcpyHostToDevice));

  project_subspace_d32_sol(c.state_num, c.d_U, c.d_X, d_srhs, d_x);
  CHECK_CUDA(cudaDeviceSynchronize());
}

}  // namespace silk::cuda
