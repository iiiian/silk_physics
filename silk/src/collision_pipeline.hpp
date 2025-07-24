#pragma once

#include <optional>
#include <vector>

#include "collision.hpp"
#include "mesh_collider.hpp"
#include "object_collider.hpp"

namespace silk {

class CollisionPipeline {
 public:
  float toi_tolerance = 0.05;
  int toi_refine_it = 10;
  float eps = 1e-6;

  std::vector<Collision> find_collision(
      std::vector<ObjectCollider>& object_colliders, float dt) const;

 private:
  std::optional<Collision> narrow_phase(const ObjectCollider& oa,
                                        const MeshCollider& ma,
                                        const ObjectCollider& ob,
                                        const MeshCollider& mb, float dt) const;
  CollisionCache<ObjectCollider> object_broadphase(
      std::vector<ObjectCollider>& obstacles) const;
};

}  // namespace silk
