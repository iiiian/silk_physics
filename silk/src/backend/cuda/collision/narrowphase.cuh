#pragma once

#include <cuda/buffer>
#include <cuda/std/span>
#include <cuda/std/utility>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

/// @brief Find the minimum point-triangle CCD time.
/// @param pt_ccache Candidate pairs.
/// @param vf_err Scene-wide numerical error bounds for point-triangle tests.
/// @param time_start Absolute normalized time already consumed.
/// @param max_time Maximum additional normalized time to search.
/// @param minimum_separation Scene-wide CCD separation distance.
/// @param tolerance Tight Inclusion solving precision.
/// @param max_iter Tight Inclusion maximum iteration count.
/// @param rt Cuda runtime.
float find_pt_min_toi(ctd::span<PTCCache> pt_ccache, const Vec3f& vf_err,
                      float time_start, float max_time,
                      float minimum_separation, float tolerance, int max_iter,
                      CudaRuntime rt);

/// @brief Find the minimum edge-edge CCD time.
/// @param ee_ccache Candidate pairs.
/// @param ee_err Scene-wide numerical error bounds for edge-edge tests.
/// @param time_start Absolute normalized time already consumed.
/// @param max_time Maximum additional normalized time to search.
/// @param minimum_separation Scene-wide CCD separation distance.
/// @param tolerance Tight Inclusion solving precision.
/// @param max_iter Tight Inclusion maximum iteration count.
/// @param rt Cuda runtime.
float find_ee_min_toi(ctd::span<EECCache> ee_ccache, const Vec3f& ee_err,
                      float time_start, float max_time,
                      float minimum_separation, float tolerance, int max_iter,
                      CudaRuntime rt);

/// @brief Append active point-triangle DCD contacts.
void find_pt_active_collisions(ctd::span<PTCCache> pt_ccache, float time,
                               float activation_distance,
                               cu::device_buffer<Collision>& output, int& fill,
                               CudaRuntime rt);

/// @brief Append active edge-edge DCD contacts.
void find_ee_active_collisions(ctd::span<EECCache> ee_ccache, float time,
                               float activation_distance,
                               cu::device_buffer<Collision>& output, int& fill,
                               CudaRuntime rt);

}  // namespace silk::cuda
