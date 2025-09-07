#pragma once

#include <Eigen/Core>

#include "ecs.hpp"

namespace silk {

/**
 * Build `ObjectCollider` for every entity that lacks one.
 *
 * Existing colliders are preserved.
 */
void make_all_object_collider(Registry& registry);

/**
 * Update all dynamic colliders from the global state vectors.
 *
 * Slices `global_state`/`prev_global_state` using each entity's
 * `SolverState::state_offset` and `state_num`, then refreshes every collider's
 * per-primitive positions, AABBs, and KD-tree.
 *
 * Args
 * - `global_state`: concatenated xyz positions for all dynamic entities.
 * - `prev_global_state`: positions from the previous step (same layout).
 */
void update_all_physical_object_collider(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& prev_global_state);

/**
 * Update all obstacle colliders from their `ObstaclePosition` component.
 *
 * Static obstacles are skipped to
 * avoid unnecessary work.
 */
void update_all_obstacle_object_collider(Registry& registry);

}  // namespace silk
