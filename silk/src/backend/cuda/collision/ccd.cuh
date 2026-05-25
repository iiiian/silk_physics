#pragma once

#include <cuda/std/numeric>
#include <cuda/std/optional>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"

namespace silk::cuda {

/// @brief Point triangle continuous collision detection.
/// @param point_collider Point collider.
/// @param triangle_collider Triangle collider.
/// @return Collision if exists. nullopt otherwise.
__device__ ctd::optional<Collision> pt_ccd(
    const PointCollider* point_collider,
    const TriangleCollider* triangle_collider);

/// @brief Edge edge continuous collision detection.
/// @param edge_collider_a Edge collider a.
/// @param edge_collider_b Edge collider b.
/// @return Collision if exists. nullopt otherwise.
__device__ ctd::optional<Collision> ee_ccd(const EdgeCollider* edge_collider_a,
                                           const EdgeCollider* edge_collider_b);

}  // namespace silk::cuda
