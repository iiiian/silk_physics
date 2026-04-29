#pragma once

#include <cuda/buffer>
#include <cuda/std/span>
#include <cuda/std/utility>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda::collision {

/// @brief Batch point triangle narrowphase.
/// @param pt_ccache Candidate pairs.
/// @param out Output collision buffer.
/// @param fill The actual size of out buffer. 0 -> empty, out.size() -> full.
/// @param rt Cuda runtime.
void pt_narrowphase(ctd::span<PTCCache> pt_ccache,
                    cu::device_buffer<Collision>& out, int& fill, CudaRuntime rt);

/// @brief Batch edge edge narrowphase.
/// @param pt_ccache Candidate pairs.
/// @param out Output collision buffer.
/// @param fill The actual size of out buffer. 0 -> empty, out.size() -> full.
/// @param rt Cuda runtime.
void ee_narrowphase(ctd::span<EECCache> ee_ccache,
                    cu::device_buffer<Collision>& out, int& fill, CudaRuntime rt);

}  // namespace silk::cuda::collision
