#pragma once

#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

/// @brief Compute inner product a dot b. Does not sync implicitly.
void inner_product(ctd::span<const float> a, ctd::span<const float> b,
                   ctd::span<float> out, CudaRuntime rt);

/// @brief Compute a dot product and a squared norm in one pass.
void inner_product_and_norm(ctd::span<const float> a, ctd::span<const float> b,
                            ctd::span<float> dot_out, ctd::span<float> norm_out,
                            CudaRuntime rt);

}  // namespace silk::cuda
