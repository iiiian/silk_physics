#pragma once

#include <cuda/std/array>
#include <cuda/std/span>
#include <vector>

#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

struct CCDQuery {
  int source_index;
  ctd::array<int, 2> state_offset;
  ctd::array<int, 4> vertex_index;
  float minimal_separation;
  ctd::array<Vec3f, 4> position_t0;
  ctd::array<Vec3f, 4> position_t1;
};

/// Build every point-triangle CCD query without applying GPU rejection.
std::vector<CCDQuery> make_pt_ccd_queries(ctd::span<PTCCache> pt_ccache,
                                          float time_start,
                                          float interval_time,
                                          float minimum_separation,
                                          CudaRuntime rt);

/// Build every edge-edge CCD query without applying GPU rejection.
std::vector<CCDQuery> make_ee_ccd_queries(ctd::span<EECCache> ee_ccache,
                                          float time_start,
                                          float interval_time,
                                          float minimum_separation,
                                          CudaRuntime rt);

/// @brief Fixed-depth Tight Inclusion rejection for point-triangle queries.
/// @return Queries that could not be rejected on the GPU.
std::vector<CCDQuery> pt_ticcd_rejection(ctd::span<PTCCache> pt_ccache,
                                         const Vec3f& err, float time_start,
                                         float interval_time,
                                         float minimum_separation,
                                         CudaRuntime rt);

/// @brief Fixed-depth Tight Inclusion rejection for edge-edge queries.
/// @return Queries that could not be rejected on the GPU.
std::vector<CCDQuery> ee_ticcd_rejection(ctd::span<EECCache> ee_ccache,
                                         const Vec3f& err, float time_start,
                                         float interval_time,
                                         float minimum_separation,
                                         CudaRuntime rt);

/// @brief Run the fixed-depth GPU rejection on an existing device query array.
/// @return Queries that could not be rejected on the GPU.
std::vector<CCDQuery> ticcd_rejection(ctd::span<const CCDQuery> device_queries,
                                      bool is_vertex_face, const Vec3f& err,
                                      CudaRuntime rt);

}  // namespace silk::cuda
