#include "backend/cuda/collision/ccd_fast_rejection.cuh"

#include <cuda/atomic>
#include <cuda/std/algorithm>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

namespace {

constexpr int WARP_SIZE = 32;
constexpr int CUDA_THREADS = 128;

__device__ void set_trajectory(const Vec3f& source_t0, const Vec3f& source_t1,
                               int state_offset, float time_start,
                               float interval_time,
                               Vec3f& position_t0, Vec3f& position_t1) {
  Vec3f delta = vsub(source_t1, source_t0);
  if (state_offset == -1) {
    position_t0 = axpby(1.0f, source_t0, time_start, delta);
    position_t1 = axpby(1.0f, position_t0, interval_time, delta);
  } else {
    position_t0 = source_t0;
    position_t1 = source_t1;
  }
}

__device__ CCDQuery make_pt_query(const PointCollider& p,
                                  const TriangleCollider& t, int source_index,
                                  float time_start, float interval_time,
                                  float minimum_separation) {
  CCDQuery query;
  query.source_index = source_index;
  query.state_offset = {p.state_offset, t.state_offset};
  query.vertex_index = {p.index, t.index(0), t.index(1), t.index(2)};
  query.minimal_separation = minimum_separation;
  set_trajectory(p.v0_t0, p.v0_t1, p.state_offset, time_start, interval_time,
                 query.position_t0[0], query.position_t1[0]);
  set_trajectory(t.v0_t0, t.v0_t1, t.state_offset, time_start, interval_time,
                 query.position_t0[1], query.position_t1[1]);
  set_trajectory(t.v1_t0, t.v1_t1, t.state_offset, time_start, interval_time,
                 query.position_t0[2], query.position_t1[2]);
  set_trajectory(t.v2_t0, t.v2_t1, t.state_offset, time_start, interval_time,
                 query.position_t0[3], query.position_t1[3]);
  return query;
}

__device__ CCDQuery make_ee_query(const EdgeCollider& ea,
                                  const EdgeCollider& eb, int source_index,
                                  float time_start, float interval_time,
                                  float minimum_separation) {
  CCDQuery query;
  query.source_index = source_index;
  query.state_offset = {ea.state_offset, eb.state_offset};
  query.vertex_index = {ea.index(0), ea.index(1), eb.index(0), eb.index(1)};
  query.minimal_separation = minimum_separation;
  set_trajectory(ea.v0_t0, ea.v0_t1, ea.state_offset, time_start, interval_time,
                 query.position_t0[0], query.position_t1[0]);
  set_trajectory(ea.v1_t0, ea.v1_t1, ea.state_offset, time_start, interval_time,
                 query.position_t0[1], query.position_t1[1]);
  set_trajectory(eb.v0_t0, eb.v0_t1, eb.state_offset, time_start, interval_time,
                 query.position_t0[2], query.position_t1[2]);
  set_trajectory(eb.v1_t0, eb.v1_t1, eb.state_offset, time_start, interval_time,
                 query.position_t0[3], query.position_t1[3]);
  return query;
}

struct FuncCoefficients {
  float c0[3];
  float c1[3];
  float c2[3];
  float c3[3];
  float c4[3];
  float c5[3];
};

template <bool is_vertex_face>
__device__ FuncCoefficients precompute_coefficients(const CCDQuery& query) {
  FuncCoefficients coefficients;
#pragma unroll
  for (int dim = 0; dim < 3; ++dim) {
    if constexpr (is_vertex_face) {
      coefficients.c0[dim] =
          query.position_t0[0](dim) - query.position_t0[1](dim);
      coefficients.c1[dim] =
          query.position_t1[0](dim) - query.position_t1[1](dim);
      coefficients.c2[dim] =
          query.position_t0[1](dim) - query.position_t0[2](dim);
      coefficients.c3[dim] =
          query.position_t1[1](dim) - query.position_t1[2](dim);
      coefficients.c4[dim] =
          query.position_t0[1](dim) - query.position_t0[3](dim);
      coefficients.c5[dim] =
          query.position_t1[1](dim) - query.position_t1[3](dim);
    } else {
      coefficients.c0[dim] =
          query.position_t0[0](dim) - query.position_t0[2](dim);
      coefficients.c1[dim] =
          query.position_t1[0](dim) - query.position_t1[2](dim);
      coefficients.c2[dim] =
          query.position_t0[1](dim) - query.position_t0[0](dim);
      coefficients.c3[dim] =
          query.position_t1[1](dim) - query.position_t1[0](dim);
      coefficients.c4[dim] =
          query.position_t0[2](dim) - query.position_t0[3](dim);
      coefficients.c5[dim] =
          query.position_t1[2](dim) - query.position_t1[3](dim);
    }
  }
  return coefficients;
}

__device__ bool vf_origin_in_bbox_1d(const FuncCoefficients& coefficients,
                                     const Vec3f& lower, const Vec3f& upper,
                                     float eps_and_ms, int dim) {
  float t_lo = lower(0);
  float t_up = upper(0);
  float u_lo = lower(1);
  float u_up = upper(1);
  float v_lo = lower(2);
  float v_up = upper(2);
  float c0 = coefficients.c0[dim];
  float c1 = coefficients.c1[dim];
  float c2 = coefficients.c2[dim];
  float c3 = coefficients.c3[dim];
  float c4 = coefficients.c4[dim];
  float c5 = coefficients.c5[dim];

  auto bounds_at_t = [&](float t, float& value_min, float& value_max) {
    float point_minus_f0 = (c1 - c0) * t + c0;
    float f0_minus_f1 = (c3 - c2) * t + c2;
    float f0_minus_f2 = (c5 - c4) * t + c4;

    if (u_up + v_up <= 1.0f) {
      float f0_minus_f1_u_lo = f0_minus_f1 * u_lo;
      float f0_minus_f1_u_up = f0_minus_f1 * u_up;
      float f0_minus_f2_v_lo = f0_minus_f2 * v_lo;
      float f0_minus_f2_v_up = f0_minus_f2 * v_up;
      value_min = point_minus_f0 +
                  ctd::min(f0_minus_f1_u_lo, f0_minus_f1_u_up) +
                  ctd::min(f0_minus_f2_v_lo, f0_minus_f2_v_up);
      value_max = point_minus_f0 +
                  ctd::max(f0_minus_f1_u_lo, f0_minus_f1_u_up) +
                  ctd::max(f0_minus_f2_v_lo, f0_minus_f2_v_up);
      return;
    }

    value_min = ctd::numeric_limits<float>::infinity();
    value_max = -ctd::numeric_limits<float>::infinity();
    auto include_boundary_point = [&](float u, float v) {
      float value = point_minus_f0 + f0_minus_f1 * u + f0_minus_f2 * v;
      value_min = ctd::min(value_min, value);
      value_max = ctd::max(value_max, value);
    };

    if (u_lo + v_lo <= 1.0f) {
      include_boundary_point(u_lo, v_lo);
    }
    if (u_lo + v_up <= 1.0f) {
      include_boundary_point(u_lo, v_up);
    }
    if (u_up + v_lo <= 1.0f) {
      include_boundary_point(u_up, v_lo);
    }
    if (u_up + v_up <= 1.0f) {
      include_boundary_point(u_up, v_up);
    }
    float v_at_u_lo = 1.0f - u_lo;
    if (v_at_u_lo >= v_lo && v_at_u_lo <= v_up) {
      include_boundary_point(u_lo, v_at_u_lo);
    }
    float v_at_u_up = 1.0f - u_up;
    if (v_at_u_up >= v_lo && v_at_u_up <= v_up) {
      include_boundary_point(u_up, v_at_u_up);
    }
    float u_at_v_lo = 1.0f - v_lo;
    if (u_at_v_lo >= u_lo && u_at_v_lo <= u_up) {
      include_boundary_point(u_at_v_lo, v_lo);
    }
    float u_at_v_up = 1.0f - v_up;
    if (u_at_v_up >= u_lo && u_at_v_up <= u_up) {
      include_boundary_point(u_at_v_up, v_up);
    }
  };

  float min_at_t_lo;
  float max_at_t_lo;
  float min_at_t_up;
  float max_at_t_up;
  bounds_at_t(t_lo, min_at_t_lo, max_at_t_lo);
  bounds_at_t(t_up, min_at_t_up, max_at_t_up);
  float min_value = ctd::min(min_at_t_lo, min_at_t_up);
  float max_value = ctd::max(max_at_t_lo, max_at_t_up);
  return min_value <= eps_and_ms && max_value >= -eps_and_ms;
}

__device__ bool ee_origin_in_bbox_1d(const FuncCoefficients& coefficients,
                                     const Vec3f& lower, const Vec3f& upper,
                                     float eps_and_ms, int dim) {
  float u_lo = lower(1);
  float u_up = upper(1);
  float v_lo = lower(2);
  float v_up = upper(2);
  float c0 = coefficients.c0[dim];
  float c1 = coefficients.c1[dim];
  float c2 = coefficients.c2[dim];
  float c3 = coefficients.c3[dim];
  float c4 = coefficients.c4[dim];
  float c5 = coefficients.c5[dim];

  auto bounds_at_t = [&](float t, float& value_min, float& value_max) {
    float ea0_minus_eb0 = (c1 - c0) * t + c0;
    float ea1_minus_ea0 = (c3 - c2) * t + c2;
    float eb0_minus_eb1 = (c5 - c4) * t + c4;
    float ea1_minus_ea0_u_lo = ea1_minus_ea0 * u_lo;
    float ea1_minus_ea0_u_up = ea1_minus_ea0 * u_up;
    float eb0_minus_eb1_v_lo = eb0_minus_eb1 * v_lo;
    float eb0_minus_eb1_v_up = eb0_minus_eb1 * v_up;
    value_min = ea0_minus_eb0 +
                ctd::min(ea1_minus_ea0_u_lo, ea1_minus_ea0_u_up) +
                ctd::min(eb0_minus_eb1_v_lo, eb0_minus_eb1_v_up);
    value_max = ea0_minus_eb0 +
                ctd::max(ea1_minus_ea0_u_lo, ea1_minus_ea0_u_up) +
                ctd::max(eb0_minus_eb1_v_lo, eb0_minus_eb1_v_up);
  };

  float min_at_t_lo;
  float max_at_t_lo;
  float min_at_t_up;
  float max_at_t_up;
  bounds_at_t(lower(0), min_at_t_lo, max_at_t_lo);
  bounds_at_t(upper(0), min_at_t_up, max_at_t_up);
  float min_value = ctd::min(min_at_t_lo, min_at_t_up);
  float max_value = ctd::max(max_at_t_lo, max_at_t_up);
  return min_value <= eps_and_ms && max_value >= -eps_and_ms;
}

/// Check if one fixed (t,u,v) leaf maps near the origin.
template <bool is_vertex_face>
__device__ bool origin_in_bbox_eval(const FuncCoefficients& coefficients,
                                    const Vec3f& lower, const Vec3f& upper,
                                    const Vec3f& err,
                                    float minimum_separation) {
#pragma unroll
  for (int dim = 0; dim < 3; ++dim) {
    float eps_and_ms = err(dim) + minimum_separation;
    bool axis_may_collide;
    if constexpr (is_vertex_face) {
      axis_may_collide =
          vf_origin_in_bbox_1d(coefficients, lower, upper, eps_and_ms, dim);
    } else {
      axis_may_collide =
          ee_origin_in_bbox_1d(coefficients, lower, upper, eps_and_ms, dim);
    }
    if (!axis_may_collide) {
      return false;
    }
  }
  return true;
}

template <bool is_vertex_face>
__device__ bool fixed_depth_rejection(const CCDQuery& query, const Vec3f& err,
                                      float max_time, int lane) {
  int t_index = lane / 16;
  int uv_index = lane % 16;
  int u_index = uv_index / 4;
  int v_index = uv_index % 4;

  float time_interval = 0.5f * max_time;
  Vec3f lower = {time_interval * t_index, 0.25f * u_index, 0.25f * v_index};
  Vec3f upper = {lower(0) + time_interval, lower(1) + 0.25f, lower(2) + 0.25f};

  if constexpr (is_vertex_face) {
    // This leaf lies entirely outside the triangle parameter domain.
    if (lower(1) + lower(2) > 1.0f) {
      return false;
    }
  }
  FuncCoefficients coefficients =
      precompute_coefficients<is_vertex_face>(query);
  return origin_in_bbox_eval<is_vertex_face>(coefficients, lower, upper, err,
                                             query.minimal_separation);
}

__global__ void reject_pt_queries(ctd::span<PTCCache> input, Vec3f err,
                                  float time_start, float interval_time,
                                  float minimum_separation,
                                  DynSpan<CCDQuery> output) {
  int global_thread = blockIdx.x * blockDim.x + threadIdx.x;
  int query_index = global_thread / WARP_SIZE;
  int lane = threadIdx.x % WARP_SIZE;
  if (query_index >= input.size()) {
    return;
  }

  auto [triangle, point] = input[query_index];
  CCDQuery query = make_pt_query(*point, *triangle, query_index, time_start,
                                 interval_time, minimum_separation);
  bool leaf_may_collide =
      fixed_depth_rejection<true>(query, err, 1.0f, lane);
  bool query_may_collide = __any_sync(0xffffffff, leaf_may_collide);

  if (lane == 0 && query_may_collide) {
    cu::atomic_ref<int> fill{*output.fill};
    int output_index = fill.fetch_add(1);
    if (output_index < output.data.size()) {
      output.data[output_index] = query;
    }
  }
}

__global__ void reject_ee_queries(ctd::span<EECCache> input, Vec3f err,
                                  float time_start, float interval_time,
                                  float minimum_separation,
                                  DynSpan<CCDQuery> output) {
  int global_thread = blockIdx.x * blockDim.x + threadIdx.x;
  int query_index = global_thread / WARP_SIZE;
  int lane = threadIdx.x % WARP_SIZE;
  if (query_index >= input.size()) {
    return;
  }

  auto [edge_a, edge_b] = input[query_index];
  CCDQuery query = make_ee_query(*edge_a, *edge_b, query_index, time_start,
                                 interval_time, minimum_separation);
  bool leaf_may_collide =
      fixed_depth_rejection<false>(query, err, 1.0f, lane);
  bool query_may_collide = __any_sync(0xffffffff, leaf_may_collide);

  if (lane == 0 && query_may_collide) {
    cu::atomic_ref<int> fill{*output.fill};
    int output_index = fill.fetch_add(1);
    if (output_index < output.data.size()) {
      output.data[output_index] = query;
    }
  }
}

template <bool is_vertex_face>
__global__ void reject_queries(ctd::span<const CCDQuery> input, Vec3f err,
                               DynSpan<CCDQuery> output) {
  int global_thread = blockIdx.x * blockDim.x + threadIdx.x;
  int query_index = global_thread / WARP_SIZE;
  int lane = threadIdx.x % WARP_SIZE;
  if (query_index >= input.size()) {
    return;
  }

  bool leaf_may_collide = fixed_depth_rejection<is_vertex_face>(
      input[query_index], err, 1.0f, lane);
  bool query_may_collide = __any_sync(0xffffffff, leaf_may_collide);
  if (lane == 0 && query_may_collide) {
    cu::atomic_ref<int> fill{*output.fill};
    int output_index = fill.fetch_add(1);
    if (output_index < output.data.size()) {
      output.data[output_index] = input[query_index];
    }
  }
}

__global__ void build_pt_queries(ctd::span<const PTCCache> candidates,
                                 float time_start, float interval_time,
                                 float minimum_separation,
                                 ctd::span<CCDQuery> queries) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= candidates.size()) {
    return;
  }
  const auto& [triangle, point] = candidates[index];
  queries[index] =
      make_pt_query(*point, *triangle, index, time_start, interval_time,
                    minimum_separation);
}

__global__ void build_ee_queries(ctd::span<const EECCache> candidates,
                                 float time_start, float interval_time,
                                 float minimum_separation,
                                 ctd::span<CCDQuery> queries) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= candidates.size()) {
    return;
  }
  const auto& [edge_a, edge_b] = candidates[index];
  queries[index] =
      make_ee_query(*edge_a, *edge_b, index, time_start, interval_time,
                    minimum_separation);
}

template <typename LaunchRejection>
std::vector<CCDQuery> run_ticcd_rejection(int query_num,
                                          LaunchRejection&& launch_rejection,
                                          CudaRuntime rt) {
  if (query_num == 0) {
    return {};
  }

  constexpr int INITIAL_CAPACITY_PERCENT = 3;
  int output_capacity =
      ctd::max(1, div_round_up(query_num * INITIAL_CAPACITY_PERCENT, 100));
  auto output = alloc<CCDQuery>(rt, output_capacity);
  auto output_fill = alloc<int>(rt, 1, 0);
  DynSpan<CCDQuery> dynamic_output{.fill = output_fill.data(), .data = output};

  int output_num;
  while (true) {
    launch_rejection(dynamic_output);
    output_num = scalar_load(output_fill.data(), rt);
    if (output_num <= output.size()) {
      break;
    }

    output = alloc<CCDQuery>(rt, output_num);
    dynamic_output.data = output;
    scalar_write(output_fill.data(), 0, rt);
  }

  return vec_like_to_host(ctd::span<const CCDQuery>(output.data(), output_num),
                          rt);
}

}  // namespace

std::vector<CCDQuery> make_pt_ccd_queries(ctd::span<PTCCache> pt_ccache,
                                          float time_start,
                                          float interval_time,
                                          float minimum_separation,
                                          CudaRuntime rt) {
  if (pt_ccache.empty()) {
    return {};
  }
  auto queries = alloc<CCDQuery>(rt, pt_ccache.size());
  int block_num = div_round_up(pt_ccache.size(), CUDA_THREADS);
  build_pt_queries<<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
      pt_ccache, time_start, interval_time, minimum_separation, queries);
  return vec_like_to_host<CCDQuery>(queries, rt);
}

std::vector<CCDQuery> make_ee_ccd_queries(ctd::span<EECCache> ee_ccache,
                                          float time_start,
                                          float interval_time,
                                          float minimum_separation,
                                          CudaRuntime rt) {
  if (ee_ccache.empty()) {
    return {};
  }
  auto queries = alloc<CCDQuery>(rt, ee_ccache.size());
  int block_num = div_round_up(ee_ccache.size(), CUDA_THREADS);
  build_ee_queries<<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
      ee_ccache, time_start, interval_time, minimum_separation, queries);
  return vec_like_to_host<CCDQuery>(queries, rt);
}

std::vector<CCDQuery> pt_ticcd_rejection(ctd::span<PTCCache> pt_ccache,
                                         const Vec3f& err, float time_start,
                                         float interval_time,
                                         float minimum_separation,
                                         CudaRuntime rt) {
  int block_num = div_round_up(pt_ccache.size(), CUDA_THREADS / WARP_SIZE);
  auto launch_rejection = [&](DynSpan<CCDQuery> output) {
    reject_pt_queries<<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
        pt_ccache, err, time_start, interval_time, minimum_separation, output);
  };
  return run_ticcd_rejection(pt_ccache.size(), launch_rejection, rt);
}

std::vector<CCDQuery> ee_ticcd_rejection(ctd::span<EECCache> ee_ccache,
                                         const Vec3f& err, float time_start,
                                         float interval_time,
                                         float minimum_separation,
                                         CudaRuntime rt) {
  int block_num = div_round_up(ee_ccache.size(), CUDA_THREADS / WARP_SIZE);
  auto launch_rejection = [&](DynSpan<CCDQuery> output) {
    reject_ee_queries<<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
        ee_ccache, err, time_start, interval_time, minimum_separation, output);
  };
  return run_ticcd_rejection(ee_ccache.size(), launch_rejection, rt);
}

std::vector<CCDQuery> ticcd_rejection(ctd::span<const CCDQuery> device_queries,
                                      bool is_vertex_face, const Vec3f& err,
                                      CudaRuntime rt) {
  int block_num = div_round_up(device_queries.size(), CUDA_THREADS / WARP_SIZE);
  auto launch_rejection = [&](DynSpan<CCDQuery> output) {
    if (is_vertex_face) {
      reject_queries<true><<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
          device_queries, err, output);
    } else {
      reject_queries<false><<<block_num, CUDA_THREADS, 0, rt.stream.get()>>>(
          device_queries, err, output);
    }
  };
  return run_ticcd_rejection(device_queries.size(), launch_rejection, rt);
}

}  // namespace silk::cuda
