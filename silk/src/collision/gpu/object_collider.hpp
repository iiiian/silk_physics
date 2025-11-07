#pragma once

#include <cuBQL/bvh.h>

#include <Eigen/Core>
#include <silk/silk.hpp>
#include <vector>

#include "collision/gpu/bvh_wrapper.hpp"
#include "collision/gpu/collision_primitive.hpp"
#include "handle.hpp"
#include "mesh.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"

namespace silk::gpu {

class GpuObjectCollider {
 public:
  Handle entity_handle;
  // If state offset = -1, this is a pure collider with no associated physics
  // state.
  int state_offset;
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
  BVHWrapper<EdgeCollider> edge_bvh;
  BVHWrapper<TriangleCollider> triangle_bvh;

 public:
  GpuObjectCollider(Handle entity_handle, const CollisionConfig& config,
                    const TriMesh& mesh, const Pin& pin,
                    const Eigen::VectorXf& mass, int state_offset);

  GpuObjectCollider(Handle entity_handle, const CollisionConfig& config,
                    const TriMesh& mesh);

  void update(const CollisionConfig& config, const ObjectState& object_state,
              const Eigen::VectorXf global_curr_state,
              const Eigen::VectorXf global_prev_state);

  void update(const CollisionConfig& config,
              const ObstaclePosition& obstacle_position);

 private:
  GpuObjectCollider() = default;
};

}  // namespace silk::gpu
