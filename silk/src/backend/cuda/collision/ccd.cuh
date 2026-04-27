#pragma once

#include <cuda/std/numeric>

#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

bool pt_ccd(const PointCollider* point_collider,
            const TriangleCollider* triangle_collider);

}  // namespace silk::cuda
