#pragma once

#include <Eigen/Core>
#include <optional>
#include <utility>

namespace silk::cpu {

/// @brief Evaluate a point on a triangle from barycentric parameters.
/// @param u Parameter for triangle vertex 2.
/// @param v Parameter for triangle vertex 3.
/// @param x1 Triangle vertex 1.
/// @param x2 Triangle vertex 2.
/// @param x3 Triangle vertex 3.
inline Eigen::Vector3f eval_triangle_parameter(float u, float v,
                                               const Eigen::Vector3f& x1,
                                               const Eigen::Vector3f& x2,
                                               const Eigen::Vector3f& x3) {
  return (1.0f - u - v) * x1 + u * x2 + v * x3;
}

/// @brief Evaluate a point on an edge from its segment parameter.
/// @param u Edge parameter.
/// @param x1 Edge vertex 1.
/// @param x2 Edge vertex 2.
inline Eigen::Vector3f eval_edge_parameter(float u, const Eigen::Vector3f& x1,
                                           const Eigen::Vector3f& x2) {
  return (1.0f - u) * x1 + u * x2;
}

/// @brief Compute barycentric parameters of a point projected on a triangle.
/// @param x0 Point position.
/// @param x1 Triangle vertex 1.
/// @param x2 Triangle vertex 2.
/// @param x3 Triangle vertex 3.
/// @param eps Epsilon for degenerate cases.
/// @return Parameters (u,v), or std::nullopt if the triangle is degenerate or
/// the projected point lies outside it.
std::optional<std::pair<float, float>> exact_pt_uv(const Eigen::Vector3f& x0,
                                                   const Eigen::Vector3f& x1,
                                                   const Eigen::Vector3f& x2,
                                                   const Eigen::Vector3f& x3,
                                                   float eps);

/// @brief Compute parameters of the closest points on two edges.
///
/// See Real-Time Collision Detection ch. 5.1.9.
///
/// @param x0 Edge 1 vertex 1.
/// @param x1 Edge 1 vertex 2.
/// @param x2 Edge 2 vertex 1.
/// @param x3 Edge 2 vertex 2.
/// @param eps Epsilon for degenerate cases.
/// @return Parameters (u,v), or std::nullopt if either edge is degenerate.
std::optional<std::pair<float, float>> exact_ee_uv(const Eigen::Vector3f& x0,
                                                   const Eigen::Vector3f& x1,
                                                   const Eigen::Vector3f& x2,
                                                   const Eigen::Vector3f& x3,
                                                   float eps);

}  // namespace silk::cpu
