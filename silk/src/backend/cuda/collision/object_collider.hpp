#pragma once

#include <Eigen/Core>

#include "backend/cuda/collision/bbox.hpp"
#include "backend/cuda/collision/broadphase.hpp"
#include "backend/cuda/collision/mesh_collider.hpp"
#include "backend/cuda/object_state.hpp"
#include "backend/cuda/obstacle_position.hpp"
#include "common/handle.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

class ObjectCollider {
 public:
  Handle entity_handle;
  // If state offset = -1, this is a pure collider with no associated physics
  // state.
  int state_offset;
  Bbox bbox;
  // If group = -1, collision is disabled. The default group should be 0.
  int group;
  bool is_static;
  bool is_self_collision_on;
  float bbox_padding;
  // Collision restitution, in range [0, 1].
  float restitution;
  // Surface friction coefficient for contact resolution.
  float friction;
  // Broadphase collision culling data struture
  KDTree<MeshCollider> mesh_collider_tree;

 public:
  /**
   * @brief Build a collider for a simulated (dynamic) object.
   *
   * @param entity_handle Owning entity handle used to look the collider up.
   * @param config Collision parameters that control filtering and materials.
   * @param mesh Geometry source; positions are copied into collider buffers.
   * @param pin Pin data identifying vertices that should not move.
   * @param mass Per-degree-of-freedom mass vector used to compute inverse mass.
   * @param state_offset Offset into the solver state array for this object.
   */
  ObjectCollider(Handle entity_handle, const CollisionConfig& config,
                 const TriMesh& mesh, const Pin& pin,
                 const Eigen::VectorXf& mass, int state_offset);

  /**
   * @brief Build a collider for a purely kinematic obstacle.
   *
   * @param entity_handle Owning entity handle used to look the collider up.
   * @param config Collision parameters that control filtering and materials.
   * @param mesh Geometry source; positions are copied into collider buffers.
   */
  ObjectCollider(Handle entity_handle, const CollisionConfig& config,
                 const TriMesh& mesh);

  /**
   * @brief Sync collider geometry with a dynamic object's current state.
   *
   * @param config Runtime collision controls (group toggles, restitution, etc).
   * @param object_state Layout information for slicing into the global state.
   * @param global_curr_state Stacked xyz positions at the end of the step.
   * @param global_prev_state Stacked xyz positions at the start of the step.
   */
  void update(const CollisionConfig& config, const ObjectState& object_state,
              const Eigen::VectorXf global_curr_state,
              const Eigen::VectorXf global_prev_state);

  /**
   * @brief Sync collider geometry with a kinematic obstacle.
   *
   * @param config Runtime collision controls (group toggles, restitution, etc).
   * @param obstacle_position Buffered obstacle poses and static state flags.
   */
  void update(const CollisionConfig& config,
              const ObstaclePosition& obstacle_position);

 private:
  ObjectCollider() = default;
};

}  // namespace silk::cuda
