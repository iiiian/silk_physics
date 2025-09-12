#pragma once

#include "bbox.hpp"
#include "collision_broadphase.hpp"
#include "handle.hpp"
#include "mesh_collider.hpp"

namespace silk {

struct ObjectCollider {
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
};

}  // namespace silk
