#pragma once

#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

/// @brief Compute inner product a dot b. Does not sync implicitly.
void inner_product(ctd::span<const float> a, ctd::span<const float> b,
                   ctd::span<float> out, CudaRuntime rt);

}  // namespace silk::cuda
