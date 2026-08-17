#include "backend/cuda/collision/dcd.cuh"

#include <cuda/atomic>
#include <cuda/std/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/optional>
#include <cuda/std/utility>

#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

namespace {

constexpr int CUDA_THREADS = 128;
constexpr float DCD_GEOMETRY_EPS = 1e-6f;

__device__ Vec3f position_at_time(const Vec3f& position_t0,
                                  const Vec3f& position_t1, float time) {
  return axpby(1.0f - time, position_t0, time, position_t1);
}

__device__ ctd::optional<Collision> make_pt_collision(
    const TriangleCollider& triangle, const PointCollider& point, float time,
    float activation_distance) {
  Vec3f x0 = position_at_time(point.v0_t0, point.v0_t1, time);
  Vec3f x1 = position_at_time(triangle.v0_t0, triangle.v0_t1, time);
  Vec3f x2 = position_at_time(triangle.v1_t0, triangle.v1_t1, time);
  Vec3f x3 = position_at_time(triangle.v2_t0, triangle.v2_t1, time);

  auto uv = exact_pt_uv(x0, x1, x2, x3, DCD_GEOMETRY_EPS);
  if (!uv) {
    return ctd::nullopt;
  }
  auto [u, v] = *uv;
  Vec3f triangle_point = eval_triangle_parameter(u, v, x1, x2, x3);
  Vec3f displacement = vsub(triangle_point, x0);
  float distance2 = dot(displacement, displacement);
  if (distance2 == 0.0f) {
    return ctd::nullopt;
  }

  float distance = ctd::sqrt(distance2);
  if (distance >= activation_distance) {
    return ctd::nullopt;
  }
  Vec3f normal = ax(-1.0f / distance, displacement);

  Vec4f parameter = {1.0f, 1.0f - u - v, u, v};
  Vec4f coefficient = {1.0f, -parameter(1), -parameter(2), -parameter(3)};
  Vec4f inv_mass = {point.inv_mass, triangle.inv_mass(0), triangle.inv_mass(1),
                    triangle.inv_mass(2)};
  float denominator = 0.0f;
#pragma unroll
  for (int i = 0; i < 4; ++i) {
    denominator += parameter(i) * parameter(i) * inv_mass(i);
  }
  if (denominator == 0.0f) {
    return ctd::nullopt;
  }
  Vec4f weight = ax(1.0f / denominator, vmul(parameter, inv_mass));
  weight(1) *= -1.0f;
  weight(2) *= -1.0f;
  weight(3) *= -1.0f;
  Vec3f correction = ax(activation_distance - distance, normal);

  Collision collision;
  collision.type = CollisionType::PointTriangle;
  collision.object_id_a = point.object_id;
  collision.object_id_b = triangle.object_id;
  collision.state_offset_a = point.state_offset;
  collision.state_offset_b = triangle.state_offset;
  collision.index = {point.index, triangle.index(0), triangle.index(1),
                     triangle.index(2)};
  collision.activation_distance = activation_distance;
  collision.friction = ctd::sqrt(point.friction * triangle.friction);
  collision.normal = normal;
  collision.coefficient = coefficient;
  collision.inv_mass = inv_mass;
  collision.x0 = x0;
  collision.x1 = x1;
  collision.x2 = x2;
  collision.x3 = x3;
  collision.prescribed_displacement = Vec3f::zeros();
  if (point.state_offset < 0) {
    collision.prescribed_displacement =
        ax(coefficient(0), vsub(point.v0_t1, x0));
  }
  if (triangle.state_offset < 0) {
    collision.prescribed_displacement = vadd(
        collision.prescribed_displacement,
        vadd(ax(coefficient(1), vsub(triangle.v0_t1, x1)),
             vadd(ax(coefficient(2), vsub(triangle.v1_t1, x2)),
                  ax(coefficient(3), vsub(triangle.v2_t1, x3)))));
  }
  collision.correction0 = ax(weight(0), correction);
  collision.correction1 = ax(weight(1), correction);
  collision.correction2 = ax(weight(2), correction);
  collision.correction3 = ax(weight(3), correction);
  return collision;
}

__device__ ctd::optional<Collision> make_ee_collision(
    const EdgeCollider& edge_a, const EdgeCollider& edge_b, float time,
    float activation_distance) {
  Vec3f x0 = position_at_time(edge_a.v0_t0, edge_a.v0_t1, time);
  Vec3f x1 = position_at_time(edge_a.v1_t0, edge_a.v1_t1, time);
  Vec3f x2 = position_at_time(edge_b.v0_t0, edge_b.v0_t1, time);
  Vec3f x3 = position_at_time(edge_b.v1_t0, edge_b.v1_t1, time);

  auto uv = exact_ee_uv(x0, x1, x2, x3, DCD_GEOMETRY_EPS);
  if (!uv) {
    return ctd::nullopt;
  }
  auto [u, v] = *uv;
  Vec3f point_a = eval_edge_parameter(u, x0, x1);
  Vec3f point_b = eval_edge_parameter(v, x2, x3);
  Vec3f displacement = vsub(point_b, point_a);
  float distance2 = dot(displacement, displacement);
  if (distance2 == 0.0f) {
    return ctd::nullopt;
  }

  float distance = ctd::sqrt(distance2);
  if (distance >= activation_distance) {
    return ctd::nullopt;
  }
  Vec3f normal = ax(-1.0f / distance, displacement);

  Vec4f parameter = {1.0f - u, u, 1.0f - v, v};
  Vec4f coefficient = {parameter(0), parameter(1), -parameter(2),
                       -parameter(3)};
  Vec4f inv_mass = {edge_a.inv_mass(0), edge_a.inv_mass(1), edge_b.inv_mass(0),
                    edge_b.inv_mass(1)};
  float denominator = 0.0f;
#pragma unroll
  for (int i = 0; i < 4; ++i) {
    denominator += parameter(i) * parameter(i) * inv_mass(i);
  }
  if (denominator == 0.0f) {
    return ctd::nullopt;
  }
  Vec4f weight = ax(1.0f / denominator, vmul(parameter, inv_mass));
  weight(2) *= -1.0f;
  weight(3) *= -1.0f;
  Vec3f correction = ax(activation_distance - distance, normal);

  Collision collision;
  collision.type = CollisionType::EdgeEdge;
  collision.object_id_a = edge_a.object_id;
  collision.object_id_b = edge_b.object_id;
  collision.state_offset_a = edge_a.state_offset;
  collision.state_offset_b = edge_b.state_offset;
  collision.index = {edge_a.index(0), edge_a.index(1), edge_b.index(0),
                     edge_b.index(1)};
  collision.activation_distance = activation_distance;
  collision.friction = ctd::sqrt(edge_a.friction * edge_b.friction);
  collision.normal = normal;
  collision.coefficient = coefficient;
  collision.inv_mass = inv_mass;
  collision.x0 = x0;
  collision.x1 = x1;
  collision.x2 = x2;
  collision.x3 = x3;
  collision.prescribed_displacement = Vec3f::zeros();
  if (edge_a.state_offset < 0) {
    collision.prescribed_displacement =
        vadd(ax(coefficient(0), vsub(edge_a.v0_t1, x0)),
             ax(coefficient(1), vsub(edge_a.v1_t1, x1)));
  }
  if (edge_b.state_offset < 0) {
    collision.prescribed_displacement = vadd(
        collision.prescribed_displacement,
        vadd(ax(coefficient(2), vsub(edge_b.v0_t1, x2)),
             ax(coefficient(3), vsub(edge_b.v1_t1, x3))));
  }
  collision.correction0 = ax(weight(0), correction);
  collision.correction1 = ax(weight(1), correction);
  collision.correction2 = ax(weight(2), correction);
  collision.correction3 = ax(weight(3), correction);
  return collision;
}

__global__ void find_pt_collisions(ctd::span<PTCCache> candidates, float time,
                                   float activation_distance,
                                   DynSpan<Collision> output) {
  int thread = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread >= candidates.size()) {
    return;
  }

  auto [triangle, point] = candidates[thread];
  auto collision =
      make_pt_collision(*triangle, *point, time, activation_distance);
  if (!collision) {
    return;
  }
  cu::atomic_ref<int> output_fill{*output.fill};
  int output_index = output_fill.fetch_add(1);
  output.data[output_index] = *collision;
}

__global__ void find_ee_collisions(ctd::span<EECCache> candidates, float time,
                                   float activation_distance,
                                   DynSpan<Collision> output) {
  int thread = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread >= candidates.size()) {
    return;
  }

  auto [edge_a, edge_b] = candidates[thread];
  auto collision =
      make_ee_collision(*edge_a, *edge_b, time, activation_distance);
  if (!collision) {
    return;
  }
  cu::atomic_ref<int> output_fill{*output.fill};
  int output_index = output_fill.fetch_add(1);
  output.data[output_index] = *collision;
}

template <typename Candidate, typename Launch>
void append_dcd_collisions(ctd::span<Candidate> candidates,
                           cu::device_buffer<Collision>& output, int& fill,
                           Launch&& launch, CudaRuntime rt) {
  if (candidates.empty()) {
    return;
  }
  int required_size = fill + candidates.size();
  if (required_size > output.size()) {
    resize_buffer(required_size, output, rt);
  }

  auto device_fill = alloc<int>(rt, 1, fill);
  DynSpan<Collision> dynamic_output{.fill = device_fill.data(), .data = output};
  launch(dynamic_output);
  fill = scalar_load(device_fill.data(), rt);
}

}  // namespace

__both__ ctd::optional<ctd::pair<float, float>> exact_pt_uv(const Vec3f& x0,
                                                            const Vec3f& x1,
                                                            const Vec3f& x2,
                                                            const Vec3f& x3,
                                                            float eps) {
  Vec3f x21 = vsub(x2, x1);
  Vec3f x31 = vsub(x3, x1);
  Vec3f x01 = vsub(x0, x1);

  float x21dx21 = dot(x21, x21);
  float x21dx31 = dot(x21, x31);
  float x31dx31 = dot(x31, x31);
  float x21dx01 = dot(x21, x01);
  float x31dx01 = dot(x31, x01);
  float det = x21dx21 * x31dx31 - x21dx31 * x21dx31;

  // Degenerate triangle; ignore.
  float area_eps = eps * ctd::max(x21dx21, x31dx31);
  if (det < area_eps * area_eps) {
    return ctd::nullopt;
  }

  // Barycentric (u, v) of point projection w.r.t. (x2, x3).
  float b1 = (x31dx31 * x21dx01 - x21dx31 * x31dx01) / det;  // U.
  float b2 = (x21dx21 * x31dx01 - x21dx31 * x21dx01) / det;  // V.
  if (b1 < 0.0f || b2 < 0.0f || b1 + b2 > 1.0f) {
    return ctd::nullopt;
  }

  return ctd::make_pair(b1, b2);
}

__both__ ctd::optional<ctd::pair<float, float>> exact_ee_uv(const Vec3f& x0,
                                                            const Vec3f& x1,
                                                            const Vec3f& x2,
                                                            const Vec3f& x3,
                                                            float eps) {
  // To handle near parallel edege better use double precision.
  double d1x = static_cast<double>(x1(0)) - x0(0);
  double d1y = static_cast<double>(x1(1)) - x0(1);
  double d1z = static_cast<double>(x1(2)) - x0(2);
  double d2x = static_cast<double>(x3(0)) - x2(0);
  double d2y = static_cast<double>(x3(1)) - x2(1);
  double d2z = static_cast<double>(x3(2)) - x2(2);
  double rx = static_cast<double>(x0(0)) - x2(0);
  double ry = static_cast<double>(x0(1)) - x2(1);
  double rz = static_cast<double>(x0(2)) - x2(2);
  double a = d1x * d1x + d1y * d1y + d1z * d1z;
  double b = d1x * d2x + d1y * d2y + d1z * d2z;
  double c = d1x * rx + d1y * ry + d1z * rz;
  double e = d2x * d2x + d2y * d2y + d2z * d2z;
  double f = d2x * rx + d2y * ry + d2z * rz;

  double eps2 = static_cast<double>(eps) * eps;
  double length_scale2 = ctd::max(a, e);
  // ignore degenerate edge.
  if (a <= eps2 * length_scale2 || e <= eps2 * length_scale2) {
    return ctd::nullopt;
  }

  double denominator = a * e - b * b;
  double u = denominator > eps2 * a * e
                 ? ctd::clamp((b * f - c * e) / denominator, 0.0, 1.0)
                 : 0.0;  // test end point for parallel edge.
  double v = (b * u + f) / e;
  if (v < 0.0) {
    v = 0.0;
    u = ctd::clamp(-c / a, 0.0, 1.0);
  } else if (v > 1.0) {
    v = 1.0;
    u = ctd::clamp((b - c) / a, 0.0, 1.0);
  }
  return ctd::make_pair(static_cast<float>(u), static_cast<float>(v));
}

void append_pt_dcd_collisions(ctd::span<PTCCache> pt_ccache, float time,
                              float activation_distance,
                              cu::device_buffer<Collision>& output, int& fill,
                              CudaRuntime rt) {
  auto launch = [&](DynSpan<Collision> dynamic_output) {
    int grid_num = div_round_up(pt_ccache.size(), CUDA_THREADS);
    find_pt_collisions<<<grid_num, CUDA_THREADS, 0, rt.stream.get()>>>(
        pt_ccache, time, activation_distance, dynamic_output);
  };
  append_dcd_collisions(pt_ccache, output, fill, launch, rt);
}

void append_ee_dcd_collisions(ctd::span<EECCache> ee_ccache, float time,
                              float activation_distance,
                              cu::device_buffer<Collision>& output, int& fill,
                              CudaRuntime rt) {
  auto launch = [&](DynSpan<Collision> dynamic_output) {
    int grid_num = div_round_up(ee_ccache.size(), CUDA_THREADS);
    find_ee_collisions<<<grid_num, CUDA_THREADS, 0, rt.stream.get()>>>(
        ee_ccache, time, activation_distance, dynamic_output);
  };
  append_dcd_collisions(ee_ccache, output, fill, launch, rt);
}

}  // namespace silk::cuda
