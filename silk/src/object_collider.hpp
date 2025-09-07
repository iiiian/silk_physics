#pragma once

#include "bbox.hpp"
#include "collision_broadphase.hpp"
#include "handle.hpp"
#include "mesh_collider.hpp"

namespace silk {

struct ObjectCollider {
  Handle entity_handle;
  // if state offset = -1, this is a pure collider
  int state_offset;
  Bbox bbox;
  // if group = -1, collision is disabled.
  // the default group should be 0.
  int group;
  bool is_obstacle;
  bool is_static;
  bool is_self_collision_on;
  float bbox_padding;
  float restitution;  // collision restitution, in range [0, 1]
  float friction;

  KDTree<MeshCollider> mesh_collider_tree;
};

}  // namespace silk
