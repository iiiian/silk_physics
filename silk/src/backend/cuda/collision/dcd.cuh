#pragma once

#include <cuda/std/optional>
#include <cuda/std/utility>

#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

/// @brief Eval triangle uv.
/// @param u param u.
/// @param v param v.
/// @param x1 triangle vertex 1.
/// @param x2 triangle vertex 2.
/// @param x3 triangle vertex 3.
__both__ Vec3f eval_triangle_parameter(float u, float v, const Vec3f& x1,
                                       const Vec3f& x2, const Vec3f& x3) {
  Vec3f tmp = axpby(1.0f - u - v, x1, u, x2);
  return axpby(1.0f, tmp, v, x3);
}

/// @brief Eval edge uv.
/// @param u param u.
/// @param v param v.
/// @param x1 edge vertex 1.
/// @param x2 edge vertex 2.
__both__ Vec3f eval_edge_parameter(float u, const Vec3f& x1, const Vec3f& x2) {
  return axpby(1.0f - u, x1, u, x2);
}

/// @brief Compute barycentric (u, v) of the point projected on the triangle.
/// @param x0 point position.
/// @param x1 triangle vertex 1.
/// @param x2 triangle vertex 2.
/// @param x3 triangle vertex 3.
/// @param eps epsilon for degenerate case.
/// @return pair (u,v) if success. nullopt if triangle is degenerate.
__both__ ctd::optional<ctd::pair<float, float>> exact_pt_uv(
    const Vec3f& x0,  // point position.
    const Vec3f& x1,  // triangle vertex 1 position.
    const Vec3f& x2,  // triangle vertex 2 position.
    const Vec3f& x3,  // triangle vertex 3 position.
    float eps);

/// @brief Compute parameters (u, v) for the closest points on two edges.
///
/// See Real-Time Collision Detection ch. 5.1.9
///
/// @param x0 point position.
/// @param x1 edge vertex 1.
/// @param x2 edge vertex 2.
/// @param eps epsilon for degenerate case.
/// @return pair (u,v) if success. nullopt if edge is degenerate.
__both__ ctd::optional<ctd::pair<float, float>> exact_ee_uv(const Vec3f& x0,
                                                            const Vec3f& x1,
                                                            const Vec3f& x2,
                                                            const Vec3f& x3,
                                                            float eps);

}  // namespace silk::cuda
