#pragma once

#include <cuda/std/numeric>
#include <cuda/std/optional>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"

namespace silk::cuda::collision {

__device__ ctd::optional<Collision> pt_ccd(
    const PointCollider* point_collider,
    const TriangleCollider* triangle_collider, float minimal_seperation,
    float restitution, float friction);

}  // namespace silk::cuda::collision
