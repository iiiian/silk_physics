#pragma once

#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

struct ADMMResidualView {
  /// ||W(Sx - z)||^2.
  float* primal_norm2;
  /// ||W * Sx||^2.
  float* primal_scale_x2;
  /// ||W * z||^2.
  float* primal_scale_aux2;
  /// rho * S^T W^T W (z_curr - z_prev).
  ctd::span<float> dual_residual;
  /// rho * S^T W^T W * z_curr.
  ctd::span<float> dual_scale_curr;
  /// rho * S^T W^T W * z_prev.
  ctd::span<float> dual_scale_prev;
};

}  // namespace silk::cuda
