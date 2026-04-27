#include <cub/cub.h>
#include <cuda_runtime.h>

#include <cuda/buffer>
#include <unordered_set>
#include <vector>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/mesh_collider.cuh"
#include "backend/cuda/obstacle_position.hpp"
#include "common/handle.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

std::vector<PointCollider> make_point_colliders(
    const TriMesh& mesh, float bbox_padding,
    std::function<float(int)> get_inv_mass) {
  auto& m = mesh;
  int point_num = m.V.rows();
  std::vector<PointCollider> point_colliders;
  for (int i = 0; i < point_num; ++i) {
    PointCollider pc;
    pc.index = i;
    pc.inv_mass = get_inv_mass(i);
    pc.v0_t0 = Vec3f::vec_like(m.V.row(i));
    pc.v0_t1 = Vec3f::vec_like(m.V.row(i));

    Bbox bbox;
    bbox.min = vmin(pc.v0_t0, pc.v0_t1);
    bbox.max = vmax(pc.v0_t0, pc.v0_t1);
    pc.bbox = Bbox::pad(bbox, bbox_padding);

    point_colliders.push_back(std::move(pc));
  }
  return point_colliders;
}

std::vector<EdgeCollider> make_edge_colliders(
    const TriMesh& mesh, float bbox_padding,
    std::function<float(int)> get_inv_mass) {
  auto& m = mesh;
  int edge_num = m.E.rows();
  std::vector<EdgeCollider> edge_colliders;
  for (int i = 0; i < edge_num; ++i) {
    EdgeCollider ec;
    ec.index = Vec2i::vec_like(m.E.row(i));
    ec.inv_mass(0) = get_inv_mass(ec.index(0));
    ec.inv_mass(1) = get_inv_mass(ec.index(1));
    ec.v0_t0 = Vec3f::vec_like(m.V.row(ec.index(0)));
    ec.v1_t0 = Vec3f::vec_like(m.V.row(ec.index(1)));
    ec.v0_t1 = Vec3f::vec_like(m.V.row(ec.index(0)));
    ec.v1_t1 = Vec3f::vec_like(m.V.row(ec.index(1)));

    Bbox bbox;
    bbox.min = vmin(vmin(ec.v0_t0, ec.v1_t0), vmin(ec.v0_t1, ec.v1_t1));
    bbox.max = vmax(vmax(ec.v0_t0, ec.v1_t0), vmax(ec.v0_t1, ec.v1_t1));
    ec.bbox = Bbox::pad(bbox, bbox_padding);

    edge_colliders.push_back(std::move(ec));
  }
  return edge_colliders;
}

std::vector<TriangleCollider> make_triangle_colliders(
    const TriMesh& mesh, float bbox_padding,
    std::function<float(int)> get_inv_mass) {
  auto& m = mesh;
  int face_num = m.F.rows();
  std::vector<TriangleCollider> triangle_colliders;
  for (int i = 0; i < face_num; ++i) {
    TriangleCollider tc;
    tc.index = Vec3i::vec_like(m.F.row(i));
    tc.inv_mass(0) = get_inv_mass(tc.index(0));
    tc.inv_mass(1) = get_inv_mass(tc.index(1));
    tc.inv_mass(2) = get_inv_mass(tc.index(2));
    tc.v0_t0 = Vec3f::vec_like(m.V.row(tc.index(0)));
    tc.v1_t0 = Vec3f::vec_like(m.V.row(tc.index(1)));
    tc.v2_t0 = Vec3f::vec_like(m.V.row(tc.index(2)));
    tc.v0_t1 = Vec3f::vec_like(m.V.row(tc.index(0)));
    tc.v1_t1 = Vec3f::vec_like(m.V.row(tc.index(1)));
    tc.v2_t1 = Vec3f::vec_like(m.V.row(tc.index(2)));

    Bbox bbox;
    Vec3f t0_min = vmin(vmin(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_min = vmin(vmin(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.min = vmin(t0_min, t1_min);
    Vec3f t0_max = vmax(vmax(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_max = vmax(vmax(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.max = vmax(t0_max, t1_max);

    tc.bbox = Bbox::pad(bbox, bbox_padding);

    triangle_colliders.push_back(std::move(tc));
  }
  return triangle_colliders;
}

ObjectCollider::ObjectCollider(Handle entity_handle,
                               const CollisionConfig& config,
                               const TriMesh& mesh, const Pin& pin,
                               const Eigen::VectorXf& mass, int state_offset,
                               CudaRuntime rt) {
  auto& c = config;
  this->entity_handle = entity_handle;
  this->state_offset = state_offset;
  // Pad object AABB by 5% of mesh scale to avoid zero-width degenerate cases.
  bbox = {.min = Vec3f::vec_like(mesh.V.colwise().minCoeff()),
          .max = Vec3f::vec_like(mesh.V.colwise().maxCoeff())};
  bbox_padding = 0.05f * mesh.avg_edge_length;
  bbox = Bbox::pad(bbox, bbox_padding);

  // Note: group == -1 disables this collider in the broad phase.
  group = (c.is_collision_on) ? c.group : -1;
  is_static = false;
  is_self_collision_on = c.is_self_collision_on;
  restitution = config.restitution;
  friction = config.friction;
  std::unordered_set<int> pin_set(pin.index.begin(), pin.index.end());
  // Pinned vertices get infinite mass (zero inverse mass) to prevent movement.
  auto get_inv_mass = [&pin_set, &mass](int index) -> float {
    if (pin_set.find(index) != pin_set.end()) {
      return 0.0f;
    }
    return 1.0f / mass(index);
  };

  auto host_triangle_colliders =
      make_triangle_colliders(mesh, bbox_padding, get_inv_mass);
  auto device_triangle_colliders =
      vec_like_to_device<TriangleCollider>(host_triangle_colliders, rt);
  triangle_collider_tree = OIBVHTree<TriangleCollider>::make(
      bbox, std::move(device_triangle_colliders), rt);

  auto host_edge_colliders =
      make_edge_colliders(mesh, bbox_padding, get_inv_mass);
  auto device_edge_colliders =
      vec_like_to_device<EdgeCollider>(host_edge_colliders, rt);
  edge_collider_tree =
      OIBVHTree<EdgeCollider>::make(bbox, std::move(device_edge_colliders), rt);

  auto host_point_colliders =
      make_point_colliders(mesh, bbox_padding, get_inv_mass);
  auto point_colliders = vec_like_to_device(host_point_colliders);

  reduced_collider_ = cu::make_buffer(rt.stream, rt.mr, 1, cu::no_init);
  cudaMemcpyAsync(&reduced_collider_.bbox, &bbox, sizeof(Bbox),
                  cudaMemcpyDefault, rt.stream.get());
}

ObjectCollider::ObjectCollider(Handle entity_handle,
                               const CollisionConfig& config,
                               const TriMesh& mesh, CudaRuntime rt) {
  auto& c = config;
  this->entity_handle = entity_handle;
  state_offset = -1;
  // Pad object AABB by 5% of mesh scale to avoid zero-width degenerate cases.
  bbox = {.min = Vec3f::vec_like(mesh.V.colwise().minCoeff()),
          .max = Vec3f::vec_like(mesh.V.colwise().maxCoeff())};
  bbox_padding = 0.05f * mesh.avg_edge_length;
  bbox = Bbox::pad(bbox, bbox_padding);

  // Note: group == -1 disables this collider in the broad phase.
  group = (c.is_collision_on) ? c.group : -1;
  is_static = false;
  is_self_collision_on = c.is_self_collision_on;
  restitution = config.restitution;
  friction = config.friction;
  // Obstacles have infinite mass (zero inverse mass).
  auto get_inv_mass = [](int) { return 0.0f; };

  auto host_triangle_colliders =
      make_triangle_colliders(mesh, bbox_padding, get_inv_mass);
  auto device_triangle_colliders =
      vec_like_to_device<TriangleCollider>(host_triangle_colliders, rt);
  triangle_collider_tree = OIBVHTree<TriangleCollider>::make(
      bbox, std::move(device_triangle_colliders), rt);

  auto host_edge_colliders =
      make_edge_colliders(mesh, bbox_padding, get_inv_mass);
  auto device_edge_colliders =
      vec_like_to_device<EdgeCollider>(host_edge_colliders, rt);
  edge_collider_tree =
      OIBVHTree<EdgeCollider>::make(bbox, std::move(device_edge_colliders), rt);

  auto host_point_colliders =
      make_point_colliders(mesh, bbox_padding, get_inv_mass);
  auto point_colliders = vec_like_to_device(host_point_colliders);

  reduced_collider_ = cu::make_buffer(rt.stream, rt.mr, 1, cu::no_init);
  cudaMemcpyAsync(&reduced_collider_.bbox, &bbox, sizeof(Bbox),
                  cudaMemcpyDefault, rt.stream.get());
}

void ObjectCollider::update(cons CollisionConfig& config,
                            ctd::span<const float> curr_state,
                            ctd::span<const float> prev_state, CudaRuntime rt) {
  // Update collision config.
  auto& c = config;
  group = (c.is_collision_on) ? c.group : -1;
  is_self_collision_on = c.is_self_collision_on;
  restitution = config.restitution;
  friction = config.friction;

  // Update triangle colliders and root bbox.
  auto triangle_colliders = triangle_collider_tree.get_colliders();
  auto update_triangles_colliders = [triangle_colliders, prev_state,
                                     curr_state] __device__(int i) {
    TriangleCollider& c = triangle_colliders[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v1_t0 = prev_state.subspan(3 * i + 3, 3);
    c.v2_t0 = prev_state.subspan(3 * i + 6, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);
    c.v1_t1 = prev_state.subspan(3 * i + 3, 3);
    c.v2_t1 = prev_state.subspan(3 * i + 6, 3);

    Bbox bbox;
    Vec3f t0_min = vmin(vmin(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_min = vmin(vmin(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.min = vmin(t0_min, t1_min);
    Vec3f t0_max = vmax(vmax(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_max = vmax(vmax(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.max = vmax(t0_max, t1_max);
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  auto merge_bbox = [] __device__(const TriangleCollider& a,
                                  const TriangleCollider& b) {
    TriangleCollider c;
    c.bbox = Bbox::merge(a.bbox, b.bbox);
    return c;
  };

  TriangleCollider init_reduce_collider;
  init_reduce_collider.bbox.min = Vec3f::max();
  init_reduce_collider.bbox.max = Vec3f::min();

  size_t temp_size;
  cub::DeviceReduce::TransformReduce(
      nullptr, temp_size, triangle_colliders.data(), reduced_collider_.data(),
      triangle_colliders.size(), merge_bbox, update_triangles_colliders,
      init_reduce_collider, rt.stream.get());

  if (device_reduce_temp_.size() < temp_size) {
    device_reduce_temp_ =
        cu::make_buffer(rt.stream, rt.mr, temp_size, cu::no_init);
  }
  cub::DeviceReduce::TransformReduce(
      device_reduce_temp_.data(), temp_size, triangle_colliders.data(),
      reduced_collider_.data(), triangle_colliders.size(), merge_bbox,
      update_triangles_colliders, init_reduce_collider, rt.stream.get());

  cudaMemcpyAsync(&bbox, &reduced_collider_.bbox, sizeof(Bbox),
                  cudaMemcpyDefault, rt.stream);

  // Update edge colliders.
  auto edge_colliders = edge_collider_tree.get_colliders();
  auto update_edge_colliders = [edge_colliders, prev_state,
                                curr_state] __device__(int i) {
    EdgeCollider& c = edge_colliders[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v1_t0 = prev_state.subspan(3 * i + 3, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);
    c.v1_t1 = prev_state.subspan(3 * i + 3, 3);

    Bbox bbox;
    bbox.min = vmin(vmin(c.v0_t0, c.v1_t0), vmin(c.v0_t1, c.v1_t1));
    bbox.max = vmax(vmax(c.v0_t0, c.v1_t0), vmax(c.v0_t1, c.v1_t1));
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  cub::DeviceFor::Bulk(edge_colliders.size(), update_edge_colliders,
                       rt.stream.get());

  // Update point colliders.
  ctd::span<PointCollider> point_colliders_span = point_colliders;
  auto update_point_colliders = [point_colliders_span, prev_state,
                                 curr_state] __device__(int i) {
    PointCollider& c = point_colliders_span[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);

    Bbox bbox;
    bbox.min = vmin(c.v0_t0, c.v0_t1);
    bbox.max = vmax(c.v0_t0, c.v0_t1);
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  cub::DeviceFor::Bulk(point_colliders.size(), update_point_colliders,
                       rt.stream.get());

  // Update BVH.
  triangle_collider_tree.update(bbox, rt);
  edge_collider_tree.update(bbox, rt);
}

void ObjectCollider::update(const CollisionConfig& config,
                            const ObstaclePosition& obstacle_position) {
  const CollisionConfig& c = config;
  const ObstaclePosition& p = obstacle_position;

  // Update collision config;
  group = (c.is_collision_on) ? c.group : -1;
  is_static = p.is_static;
  restitution = config.restitution;
  friction = config.friction;
  // If obstacle stays static for 2+ steps, there's no need to update mesh
  // colliders.
  if (p.is_static_twice) {
    return;
  }
  // If obstacle is static, position at t0 is the current obstacle position.
  auto& position_t0 = (p.is_static) ? p.curr_position : p.prev_position;
  ctd::span<const float> prev_state{position_t0.data(), position_t0.size()};
  ctd::span<const float> curr_state{p.curr_position.data(),
                                    p.curr_position.size()};

  // Update triangle colliders and root bbox.
  auto triangle_colliders = triangle_collider_tree.get_colliders();
  auto update_triangles_colliders = [triangle_colliders, prev_state,
                                     curr_state] __device__(int i) {
    TriangleCollider& c = triangle_colliders[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v1_t0 = prev_state.subspan(3 * i + 3, 3);
    c.v2_t0 = prev_state.subspan(3 * i + 6, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);
    c.v1_t1 = prev_state.subspan(3 * i + 3, 3);
    c.v2_t1 = prev_state.subspan(3 * i + 6, 3);

    Bbox bbox;
    Vec3f t0_min = vmin(vmin(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_min = vmin(vmin(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.min = vmin(t0_min, t1_min);
    Vec3f t0_max = vmax(vmax(tc.v0_t0, tc.v1_t0), tc.v2_t0);
    Vec3f t1_max = vmax(vmax(tc.v0_t1, tc.v1_t1), tc.v2_t1);
    bbox.max = vmax(t0_max, t1_max);
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  auto merge_bbox = [] __device__(const TriangleCollider& a,
                                  const TriangleCollider& b) {
    TriangleCollider c;
    c.bbox = Bbox::merge(a.bbox, b.bbox);
    return c;
  };

  TriangleCollider init_reduce_collider;
  init_reduce_collider.bbox.min = Vec3f::max();
  init_reduce_collider.bbox.max = Vec3f::min();

  size_t temp_size;
  cub::DeviceReduce::TransformReduce(
      nullptr, temp_size, triangle_colliders.data(), reduced_collider_.data(),
      triangle_colliders.size(), merge_bbox, update_triangles_colliders,
      init_reduce_collider, rt.stream.get());

  if (device_reduce_temp_.size() < temp_size) {
    device_reduce_temp_ =
        cu::make_buffer(rt.stream, rt.mr, temp_size, cu::no_init);
  }
  cub::DeviceReduce::TransformReduce(
      device_reduce_temp_.data(), temp_size, triangle_colliders.data(),
      reduced_collider_.data(), triangle_colliders.size(), merge_bbox,
      update_triangles_colliders, init_reduce_collider, rt.stream.get());

  cudaMemcpyAsync(&bbox, &reduced_collider_.bbox, sizeof(Bbox),
                  cudaMemcpyDefault, rt.stream);

  // Update edge colliders.
  auto edge_colliders = edge_collider_tree.get_colliders();
  auto update_edge_colliders = [edge_colliders, prev_state,
                                curr_state] __device__(int i) {
    EdgeCollider& c = edge_colliders[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v1_t0 = prev_state.subspan(3 * i + 3, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);
    c.v1_t1 = prev_state.subspan(3 * i + 3, 3);

    Bbox bbox;
    bbox.min = vmin(vmin(c.v0_t0, c.v1_t0), vmin(c.v0_t1, c.v1_t1));
    bbox.max = vmax(vmax(c.v0_t0, c.v1_t0), vmax(c.v0_t1, c.v1_t1));
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  cub::DeviceFor::Bulk(edge_colliders.size(), update_edge_colliders,
                       rt.stream.get());

  // Update point colliders.
  ctd::span<PointCollider> point_colliders_span = point_colliders;
  auto update_point_colliders = [point_colliders_span, prev_state,
                                 curr_state] __device__(int i) {
    PointCollider& c = point_colliders_span[i];
    c.v0_t0 = prev_state.subspan(3 * i, 3);
    c.v0_t1 = prev_state.subspan(3 * i, 3);

    Bbox bbox;
    bbox.min = vmin(c.v0_t0, c.v0_t1);
    bbox.max = vmax(c.v0_t0, c.v0_t1);
    c.bbox = Bbox::pad(bbox, bbox_padding);
  };
  cub::DeviceFor::Bulk(point_colliders.size(), update_point_colliders,
                       rt.stream.get());

  // Update BVH.
  triangle_collider_tree.update(bbox, rt);
  edge_collider_tree.update(bbox, rt);
}

}  // namespace silk::cuda
