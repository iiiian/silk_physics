#pragma once

#include <optional>

#include "backend/cuda/collision/collision.hpp"
#include "backend/cuda/collision/mesh_collider.hpp"
#include "backend/cuda/collision/object_collider.hpp"

namespace silk::cuda {

/**
 * Compute the earliest narrow-phase collision between two mesh collider
 * pairs over the interval [0, dt]. Uses Tight Inclusion CCD to assemble a
 * Collision with time of impact, per-vertex impulse weights, and post-collision
 * velocity deltas.
 *
 * @param oa Object collider for A.
 * @param ma Mesh collider for A.
 * @param ob Object collider for B.
 * @param mb Mesh collider for B.
 * @param dt Time step length.
 * @param base_stiffness Base penalty stiffness assigned to a new collision.
 * @param min_toi Minimum enforced time of impact to avoid zero-TOI stalls.
 * @param tolerance Tight inclusion CCD tolerance.
 * @param max_iter Tight inclusion maximum CCD iterations; pass -1 to disable
 * the limit.
 * @param scene_ee_err Scene-wide numerical error bounds for edge–edge tests.
 * @param scene_vf_err Scene-wide numerical error bounds for vertex–face tests.
 * @return Populated Collision or std::nullopt if no collision is detected.
 */
std::optional<Collision> narrow_phase(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float min_toi,
    float tolerance, int max_iter, const Eigen::Array3f& scene_ee_err,
    const Eigen::Array3f& scene_vf_err);

/**
 * Re-evaluate an existing collision with updated positions and adjust
 * stiffness. Updates positions from the solver states, runs a low-iteration
 * CCD recheck, and grows stiffness while the collision persists.
 *
 * @param solver_state_t0 Positions at the start of the step (x, y, z stacked).
 * @param solver_state_t1 Positions at the end of the step.
 * @param scene_ee_err Error bound for edge–edge CCD.
 * @param scene_vf_err Error bound for vertex–face CCD.
 * @param base_stiffness Initial stiffness when a collision appears.
 * @param max_stiffness Upper bound on stiffness growth.
 * @param growth_factor Multiplicative growth factor when colliding.
 * @param tolerance Tight inclusion CCD tolerance.
 * @param max_iter Tight inclusion maximum CCD iterations for the partial
 * recheck.
 * @param collision Collision to update in place.
 */
void partial_ccd_update(const Eigen::VectorXf& solver_state_t0,
                        const Eigen::VectorXf& solver_state_t1,
                        const Eigen::Array3f& scene_ee_err,
                        const Eigen::Array3f& scene_vf_err,
                        float base_stiffness, float max_stiffness,
                        float growth_factor, float tolerance, int max_iter,
                        Collision& collision);

}  // namespace silk::cuda
