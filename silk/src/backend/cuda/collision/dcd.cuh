#pragma once

#include <cuda/std/optional>
#include <cuda/std/utility>

#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Compute barycentric (u, v) of the point projected on the triangle.
__both__ ctd::optional<ctd::pair<float, float>> exact_point_triangle_uv(
    Mat34fV position, float eps);

/// @brief Compute parameters (u, v) for the closest points on two edges.
///
/// See Real-Time Collision Detection ch. 5.1.9
__both__ ctd::optional<ctd::pair<float, float>> exact_edge_edge_uv(
    Mat34fV position, float eps);

}  // namespace silk::cuda
