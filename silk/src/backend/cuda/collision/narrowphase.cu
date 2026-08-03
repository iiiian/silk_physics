#include "backend/cuda/collision/narrowphase.cuh"

#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <atomic>
#include <cuda/buffer>
#include <cuda/std/span>
#include <optional>
#include <vector>

#include "backend/cpu/collision/ccd.hpp"
#include "backend/cuda/collision/ccd_fast_rejection.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

namespace {

Eigen::Vector3f to_eigen(const Vec3f& value) {
  return {value(0), value(1), value(2)};
}

Eigen::Array3f to_eigen_array(const Vec3f& value) {
  return {value(0), value(1), value(2)};
}

template <bool is_vertex_face>
float solve_min_toi(ctd::span<const CCDQuery> queries, const Vec3f& err,
                    float max_time, float tolerance, int max_iter) {
  std::atomic<float> earliest_toi{max_time};
  Eigen::Array3f numerical_error = to_eigen_array(err);
  int query_num = queries.size();
  tbb::parallel_for(0, query_num, [&](int i) {
    float query_max_time = earliest_toi.load(std::memory_order_relaxed);
    if (query_max_time <= 0.0f) {
      return;
    }

    const CCDQuery& query = queries[i];
    std::optional<cpu::CCDResult> result;
    if constexpr (is_vertex_face) {
      result = cpu::vertex_face_ccd(
          to_eigen(query.position_t0[0]), to_eigen(query.position_t0[1]),
          to_eigen(query.position_t0[2]), to_eigen(query.position_t0[3]),
          to_eigen(query.position_t1[0]), to_eigen(query.position_t1[1]),
          to_eigen(query.position_t1[2]), to_eigen(query.position_t1[3]),
          numerical_error, query.minimal_separation, tolerance, max_iter, true,
          query_max_time);
    } else {
      result = cpu::edge_edge_ccd(
          to_eigen(query.position_t0[0]), to_eigen(query.position_t0[1]),
          to_eigen(query.position_t0[2]), to_eigen(query.position_t0[3]),
          to_eigen(query.position_t1[0]), to_eigen(query.position_t1[1]),
          to_eigen(query.position_t1[2]), to_eigen(query.position_t1[3]),
          numerical_error, query.minimal_separation, tolerance, max_iter, true,
          query_max_time);
    }
    if (!result) {
      return;
    }

    float toi = result->use_small_ms ? result->small_ms_t(0) : result->t(0);
    if (toi == 0.0f) {
      return;
    }
    float previous = earliest_toi.load(std::memory_order_relaxed);
    while (toi < previous && !earliest_toi.compare_exchange_weak(
                                 previous, toi, std::memory_order_relaxed)) {
    }
  });
  return earliest_toi.load(std::memory_order_relaxed);
}

}  // namespace

float find_pt_min_toi(ctd::span<PTCCache> pt_ccache, const Vec3f& vf_err,
                      float time_start, float max_time,
                      float minimum_separation, float tolerance, int max_iter,
                      CudaRuntime rt) {
  std::vector<CCDQuery> unresolved = pt_ticcd_rejection(
      pt_ccache, vf_err, time_start, max_time, minimum_separation, rt);
  return solve_min_toi<true>(unresolved, vf_err, max_time, tolerance, max_iter);
}

float find_ee_min_toi(ctd::span<EECCache> ee_ccache, const Vec3f& ee_err,
                      float time_start, float max_time,
                      float minimum_separation, float tolerance, int max_iter,
                      CudaRuntime rt) {
  std::vector<CCDQuery> unresolved = ee_ticcd_rejection(
      ee_ccache, ee_err, time_start, max_time, minimum_separation, rt);
  return solve_min_toi<false>(unresolved, ee_err, max_time, tolerance,
                              max_iter);
}

void find_pt_active_collisions(ctd::span<PTCCache> pt_ccache, float time,
                               float activation_distance,
                               cu::device_buffer<Collision>& output, int& fill,
                               CudaRuntime rt) {
  append_pt_dcd_collisions(pt_ccache, time, activation_distance, output, fill,
                           rt);
}

void find_ee_active_collisions(ctd::span<EECCache> ee_ccache, float time,
                               float activation_distance,
                               cu::device_buffer<Collision>& output, int& fill,
                               CudaRuntime rt) {
  append_ee_dcd_collisions(ee_ccache, time, activation_distance, output, fill,
                           rt);
}

}  // namespace silk::cuda
