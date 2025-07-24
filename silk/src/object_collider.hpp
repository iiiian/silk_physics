#pragma once

#include "bbox.hpp"
#include "collision_broadphase.hpp"
#include "mesh_collider.hpp"

namespace silk {

struct ObjectCollider {
  Bbox bbox;
  // if group = -1, collision is disabled
  // the default group should be 0
  int group;
  // if solver_offset = -1, this is a pure collider
  int solver_offset;
  bool is_static;
  bool is_self_collision_on;
  float bbox_padding;
  float damping;
  float friction;
  std::vector<MeshCollider> mesh_colliders;
  KDTree<MeshCollider> mesh_collider_tree;
};

}  // namespace silk
