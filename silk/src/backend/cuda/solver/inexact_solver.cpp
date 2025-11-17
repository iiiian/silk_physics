#include "backend/cuda/solver/inexact_solver.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/device_vector.hpp"
#include "backend/cuda/solver/cloth_solver_context.hpp"
#include "backend/cuda/solver/inexact_solver_kernel.hpp"

namespace silk::cuda {

void inexact_solve(const ClothSolverContext& solver_context, const float* d_rhs,
                   float* d_x) {
  auto& c = solver_context;

  DVector<float> d_buffer{32};
  compute_subspace_d32_rhs(c.state_num, c.d_U, c.d_HX, d_rhs, d_buffer);
  CHECK_CUDA(cudaDeviceSynchronize());

  Eigen::Vector<float, 32> h_srhs;
  CHECK_CUDA(cudaMemcpy(h_srhs.data(), d_buffer, 32 * sizeof(float),
                        cudaMemcpyDeviceToHost));
  Eigen::Vector<float, 32> h_ssol = c.UHU.ldlt().solve(h_srhs);
  CHECK_CUDA(cudaMemcpy(d_buffer, h_ssol.data(), 32 * sizeof(float),
                        cudaMemcpyHostToDevice));

  project_subspace_d32_sol(c.state_num, c.d_U, c.d_X, d_buffer, d_x);
  CHECK_CUDA(cudaDeviceSynchronize());
}

}  // namespace silk::cuda
