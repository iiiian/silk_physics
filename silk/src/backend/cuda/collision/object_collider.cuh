#pragma once

#include <Eigen/Core>
#include <cuda/std/span>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/pin.hpp"
#include "common/config_plus.hpp"
#include "common/initial_state.hpp"
#include "common/mesh.hpp"

namespace silk::cuda {

class ObjectCollider {
 public:
  Bbox bbox;
  // group = 0  -> default group
  // group = -1 -> collision disabled
  int group;
  bool is_static;
  bool is_self_collision_on;
  bool is_physical;
  float bbox_padding;
  // Broadphase collision culling data struture
  Buf<PointCollider> point_colliders;
  OIBVHTree<TriangleCollider> triangle_collider_tree;
  OIBVHTree<EdgeCollider> edge_collider_tree;

 public:
  ObjectCollider() = default;

  /// @brief Build a collider for physical object.
  ///
  /// @param[in] config Collision config.
  /// @param[in] mesh Object mesh.
  /// @param[in] pin Object's pinned vertex indices.
  /// @param[in] mass Object vertex mass vector.
  /// @param[in] state_offset Object offset in global state.
  static ObjectCollider from_physical(const CollisionConfigPlus& config,
                                      const TriMesh& mesh, const PinIndex& pin,
                                      const InitialState& init_state,
                                      const Eigen::VectorXf& mass,
                                      int state_offset, CudaRuntime rt);

  /// @brief Build a collider for pure obstacle.
  ///
  /// @param[in] config Collision config.
  /// @param[in] mesh Object mesh.
  static ObjectCollider from_obstacle(const CollisionConfigPlus& config,
                                      const TriMesh& mesh,
                                      const InitialState& init_state,
                                      CudaRuntime rt);

  /// @brief Update collider collision config.
  void update_collision_config(const CollisionConfigPlus& config,
                               CudaRuntime rt);

  /// @brief Update collider state (position).
  void update_position(ctd::span<const float> curr_state,
                       ctd::span<const float> prev_state, CudaRuntime rt);

  /// @brief Update collider state offset.
  void update_state_offset(int state_offset, CudaRuntime rt);

  /// @brief Update collider collision config, position, and state.
  ///
  /// @param[in] config Collision config.
  /// @param[in] curr_state state vector.
  /// @param[in] prev_state state vector.
  /// @param[in] state_offset Object offset in global state. -1 means obstacle.
  void update_all(const CollisionConfigPlus& config,
                  ctd::span<const float> curr_state,
                  ctd::span<const float> prev_state, int state_offset,
                  CudaRuntime rt);

 private:
  // Temp data for object collider bbox reduction.
  Buf<char> device_reduce_temp_;
  Buf<TriangleCollider> reduced_collider_;
};

}  // namespace silk::cuda
