#pragma once

#include <Eigen/Core>
#include <silk/silk.hpp>

#include "handle.hpp"
#include "mesh.hpp"
#include "object_collider.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"

namespace silk {

/**
 * Creates a collision object for a physical entity.
 * Pinned vertices get infinite mass to prevent movement.
 */
ObjectCollider make_physical_object_collider(
    Handle entity_handle, const CollisionConfig& config, const TriMesh& mesh,
    const Pin& pin, const Eigen::VectorXf& mass, int state_offset);

/**
 * Creates a collision object for an obstacle with infinite mass.
 * Obstacles don't respond to forces but can move via prescribed motion.
 */
ObjectCollider make_obstacle_object_collider(Handle entity_handle,
                                             const CollisionConfig& config,
                                             const TriMesh& mesh);

void update_physical_object_collider(const CollisionConfig& config,
                                     const ObjectState& object_state,
                                     const Eigen::VectorXf global_curr_state,
                                     const Eigen::VectorXf global_prev_state,
                                     ObjectCollider& object_collider);

void update_obstacle_object_collider(const CollisionConfig& config,
                                     const ObstaclePosition& obstacle_position,
                                     ObjectCollider& object_collider);

}  // namespace silk
