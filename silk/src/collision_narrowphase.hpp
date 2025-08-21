#pragma once

#include <optional>

#include "collision.hpp"
#include "mesh_collider.hpp"
#include "object_collider.hpp"

namespace silk {

std::optional<Collision> narrow_phase(
    const ObjectCollider& oa, const MeshCollider& ma, const ObjectCollider& ob,
    const MeshCollider& mb, float dt, float base_stiffness, float tolerance,
    int max_iter, const Eigen::Array3f& scene_ee_err,
    const Eigen::Array3f& scene_vf_err);

void partial_ccd_update(const Eigen::VectorXf& solver_state_t0,
                        const Eigen::VectorXf& solver_state_t1,
                        const Eigen::Array3f& scene_ee_err,
                        const Eigen::Array3f& scene_vf_err,
                        float base_stiffness, float max_stiffness,
                        float growth_factor, float tolerance, int max_iter,
                        Collision& collision);

}  // namespace silk
