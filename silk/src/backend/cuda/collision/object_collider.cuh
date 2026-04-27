#pragma once

#include <Eigen/Core>
#include <cuda/std/span>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/object_state.cuh"
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
  Buf<PointCollider> point_colliders;
  OIBVHTree<TriangleCollider> triangle_collider_tree;
  OIBVHTree<EdgeCollider> edge_collider_tree;

 public:
  /// @brief Build a collider for dynamic object.
  ///
  /// @param[in] entity_handle ECS entity handle.
  /// @param[in] config Collision config.
  /// @param[in] mesh Object mesh.
  /// @param[in] pin Object's pinned vertex indices.
  /// @param[in] mass Object vertex mass vector.
  /// @param[in] state_offset Object state offset into global state vector.
  ObjectCollider(Handle entity_handle, const CollisionConfig& config,
                 const TriMesh& mesh, const Pin& pin,
                 const Eigen::VectorXf& mass, int state_offset, CudaRuntime rt);

  /// @brief Build a collider for pure obstacle.
  ///
  /// @param[in] entity_handle ECS entity handle.
  /// @param[in] config Collision config.
  /// @param[in] mesh Object mesh.
  ObjectCollider(Handle entity_handle, const CollisionConfig& config,
                 const TriMesh& mesh, CudaRuntime rt);

  /// @brief Update collider from collision config and global state.
  ///
  /// @param[in] config Collision config.
  /// @param[in] global_curr_state Global state vector.
  /// @param[in] global_prev_state Global state vector.
  void update(const CollisionConfig& config, ctd::span<const float> curr_state,
              ctd::span<const float> prev_state, CudaRuntime rt);

  /// @brief Update collider from collision config and obstacle position.
  ///
  /// @param[in] config Collision config.
  /// @param[in] obstacle_position Obstacle position.
  void update(const CollisionConfig& config,
              const ObstaclePosition& obstacle_position);

 private:
  // Temp data for object collider bbox reduction.
  Buf<char> device_reduce_temp_;
  Buf<TriangleCollider> reduced_collider_;

  ObjectCollider() = default;
};

}  // namespace silk::cuda
