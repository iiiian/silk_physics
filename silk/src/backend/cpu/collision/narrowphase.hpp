#pragma once

#include <optional>

#include "backend/cpu/collision/collision.hpp"
#include "backend/cpu/collision/mesh_collider.hpp"
#include "backend/cpu/collision/object_collider.hpp"

namespace silk::cpu {

/// Compute the earliest CCD time for one primitive pair.
///
/// @param oa Object collider for A.
/// @param ma Mesh collider for A.
/// @param ob Object collider for B.
/// @param mb Mesh collider for B.
/// @param time_start Normalizd starting time in current step.
/// @param max_time Upper bound of the relative normalized time interval.
/// @param minimum_separation CCD-only minimum separation distance.
/// @param tolerance Tight inclusion CCD tolerance.
/// @param max_iter Tight inclusion maximum CCD iterations; -1 = no limit.
/// @param scene_ee_err Scene-wide numerical error bounds for edge–edge tests.
/// @param scene_vf_err Scene-wide numerical error bounds for vertex–face tests.
/// @return Conservative earliest TOI or std::nullopt if no hit is detected.
// clang-format off
std::optional<float> find_min_toi(
    const ObjectCollider& oa,
    const MeshCollider& ma,
    const ObjectCollider& ob,
    const MeshCollider& mb,
    float time_start,
    float max_time,
    float minimum_separation,
    float tolerance,
    int max_iter,
    const Eigen::Array3f& scene_ee_err,
    const Eigen::Array3f& scene_vf_err);
// clang-format on

/// Run DCD for one primitive pair at a fixed normalized frame time.
/// @param time Absolute normalized time in [0,1].
/// @param activation_distance DCD-only contact activation distance.
/// @param stiffness Penalty stiffness assigned to the active contact.
/// @return Active contact or std::nullopt when the distance is outside the
/// activation threshold.
std::optional<Collision> find_active_collision(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float time, float activation_distance,
    float stiffness);

}  // namespace silk::cpu
