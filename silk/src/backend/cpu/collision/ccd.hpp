#pragma once

#include <Eigen/Core>
#include <optional>

namespace silk::cpu {

/// @brief Result of continuous collision detection over t ∈ [0, 1].
///
/// The time interval stores conservative bounds for the earliest time of
/// impact. Units follow input space.
struct CCDResult {
  Eigen::Array2f t;   // Time-of-impact interval [t_min, t_max] within [0, 1].
  float tolerance;    // Co-domain tolerance used at termination.
  bool use_small_ms;  // True if a smaller minimum separation was used.
  Eigen::Array2f small_ms_t;  // TOI interval obtained with the refined ms.
};

/// @brief Edge–edge CCD with minimum separation and earliest TOI.
///
/// Evaluates motion from t=0 to t=1 and returns a conservative TOI interval
/// if an intersection occurs.
///
/// @param[in] ea0_t0 First edge v0 at t=0.
/// @param[in] ea1_t0 First edge v1 at t=0.
/// @param[in] eb0_t0 Second edge v0 at t=0.
/// @param[in] eb1_t0 Second edge v1 at t=0.
/// @param[in] ea0_t1 First edge v0 at t=1.
/// @param[in] ea1_t1 First edge v1 at t=1.
/// @param[in] eb0_t1 Second edge v0 at t=1.
/// @param[in] eb1_t1 Second edge v1 at t=1.
/// @param[in] err Per-axis numerical filter from scene bounds.
/// @param[in] ms Minimum separation distance for CCD.
/// @param[in] tolerance Distance tolerance used to bound co-domain (typ. 1e-6).
/// @param[in] max_iter Max bisection iterations; -1 disables early stop.
/// @param[in] no_zero_toi If true, refines to avoid returning t=0 when
/// possible.
/// @param[in] max_time Upper bound of the searched time interval.
/// @return Populated CCDResult on collision; std::nullopt otherwise.
std::optional<CCDResult> edge_edge_ccd(
    const Eigen::Vector3f &ea0_t0, const Eigen::Vector3f &ea1_t0,
    const Eigen::Vector3f &eb0_t0, const Eigen::Vector3f &eb1_t0,
    const Eigen::Vector3f &ea0_t1, const Eigen::Vector3f &ea1_t1,
    const Eigen::Vector3f &eb0_t1, const Eigen::Vector3f &eb1_t1,
    const Eigen::Array3f &err, float ms, float tolerance, long max_iter,
    bool no_zero_toi, float max_time = 1.0f);

/// @brief Vertex–face CCD with minimum separation and earliest TOI.
///
/// Evaluates motion from t=0 to t=1 and returns a conservative TOI interval
/// if the vertex approaches the triangle within separation ms.
///
/// @param[in] v_t0 Vertex at t=0.
/// @param[in] f0_t0 Face v0 at t=0.
/// @param[in] f1_t0 Face v1 at t=0.
/// @param[in] f2_t0 Face v2 at t=0.
/// @param[in] v_t1 Vertex at t=1.
/// @param[in] f0_t1 Face v0 at t=1.
/// @param[in] f1_t1 Face v1 at t=1.
/// @param[in] f2_t1 Face v2 at t=1.
/// @param[in] err Per-axis numerical filter from scene bounds.
/// @param[in] ms Minimum separation distance for CCD.
/// @param[in] tolerance Distance tolerance used to bound co-domain (typ. 1e-6).
/// @param[in] max_iter Max bisection iterations; -1 disables early stop.
/// @param[in] no_zero_toi If true, refines to avoid returning t=0 when
/// possible.
/// @param[in] max_time Upper bound of the searched time interval.
/// @return Populated CCDResult on collision; std::nullopt otherwise.
std::optional<CCDResult> vertex_face_ccd(
    const Eigen::Vector3f &v_t0, const Eigen::Vector3f &f0_t0,
    const Eigen::Vector3f &f1_t0, const Eigen::Vector3f &f2_t0,
    const Eigen::Vector3f &v_t1, const Eigen::Vector3f &f0_t1,
    const Eigen::Vector3f &f1_t1, const Eigen::Vector3f &f2_t1,
    const Eigen::Array3f &err, float ms, float tolerance, long max_iter,
    bool no_zero_toi, float max_time = 1.0f);

}  // namespace silk::cpu
