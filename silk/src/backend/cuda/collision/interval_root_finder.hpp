#pragma once

#include <array>
#include <optional>

#include "backend/cuda/collision/ccd.hpp"
#include "backend/cuda/collision/interval.hpp"

namespace silk::cuda {

/// @brief Interval root finding (BFS) over (t,u,v) for CCD.
/// @param[in] a_t0 Vertex a at t=0.
/// @param[in] b_t0 Vertex b at t=0.
/// @param[in] c_t0 Vertex c at t=0.
/// @param[in] d_t0 Vertex d at t=0.
/// @param[in] a_t1 Vertex a at t=1.
/// @param[in] b_t1 Vertex b at t=1.
/// @param[in] c_t1 Vertex c at t=1.
/// @param[in] d_t1 Vertex d at t=1.
/// @param[in] iset Initial domain for (t,u,v), typically [0,1]^3.
/// @param[in] tol Per-parameter splitting tolerances for (t,u,v).
/// @param[in] co_domain_tolerance Distance tolerance used in bbox evaluation.
/// @param[in] err Per-axis numerical filter (scene-based error bounds).
/// @param[in] ms Minimum separation distance.
/// @param[in] max_iter Maximum iterations; -1 disables early termination.
/// @param[in] is_unit_interval If true, uses a fast unit-domain evaluation
/// once.
/// @tparam is_vertex_face Switch between vertex–face and edge–edge.
/// @return CCDResult if a root is found; std::nullopt otherwise.
template <bool is_vertex_face>
std::optional<CCDResult> interval_root_finder_BFS(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval);

/// @brief Compute numeric filter for CCD from scene magnitude.
/// @param abs_max Per-axis absolute coordinate maxima used for scaling.
/// @param is_vertex_face True for VF queries; false for EE.
/// @param using_minimum_separation Whether minimum separation is enabled.
Eigen::Array3f get_numerical_error(const Eigen::Vector3f &abs_max,
                                   bool is_vertex_face);

}  // namespace silk::cuda
