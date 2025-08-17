#pragma once

#include <optional>

#include "collision.hpp"
#include "mesh_collider.hpp"
#include "object_collider.hpp"

namespace silk {

std::optional<Collision> narrow_phase(const ObjectCollider& oa,
                                      const MeshCollider& ma,
                                      const ObjectCollider& ob,
                                      const MeshCollider& mb, float dt,
                                      const Eigen::Array3f& scene_ee_err,
                                      const Eigen::Array3f& scene_vf_err);

}
