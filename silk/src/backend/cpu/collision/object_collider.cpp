#include "backend/cpu/collision/object_collider.hpp"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <functional>
#include <unordered_set>

#include "backend/cpu/obstacle_position.hpp"
#include "common/handle.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cpu {

/// Creates all mesh colliders (vertices, edges, faces) for collision detection.
///
/// @param mesh Triangle mesh to convert into collision primitives
/// @param bbox_padding Amount to expand bounding boxes for collision tolerance
/// @param get_inv_mass Function returning inverse mass for each vertex index
/// @return Vector of mesh colliders ready for spatial partitioning
///
/// Creates three types of colliders for comprehensive collision detection:
/// - Point colliders for vertex-vertex and vertex-face interactions
/// - Edge colliders for edge-edge interactions
/// - Triangle colliders for continuous collision detection against faces
static std::vector<MeshCollider> make_mesh_colliders(
    const TriMesh& mesh, float bbox_padding,
    std::function<float(int)> get_inv_mass) {
  auto& m = mesh;
  int vert_num = m.V.rows();
  int edge_num = m.E.rows();
  int face_num = m.F.rows();
  std::vector<MeshCollider> mesh_colliders;
  // Create point colliders for vertex-level collision detection.
  for (int i = 0; i < vert_num; ++i) {
    MeshCollider mc;
    mc.type = MeshColliderType::Point;
    mc.index(0) = i;
    mc.inv_mass(0) = get_inv_mass(i);
    mc.position_t0.col(0) = m.V.row(i);
    mc.position_t1.col(0) = m.V.row(i);
    mc.bbox.min = m.V.row(i);
    mc.bbox.max = m.V.row(i);
    mc.bbox.pad_inplace(bbox_padding);
    mesh_colliders.emplace_back(std::move(mc));
  }
  // Create edge colliders for edge-edge collision detection.
  for (int i = 0; i < edge_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;
    mc.type = MeshColliderType::Edge;
    mc.index(Eigen::seqN(0, 2)) = m.E.row(i);
    mc.inv_mass(0) = get_inv_mass(mc.index(0));
    mc.inv_mass(1) = get_inv_mass(mc.index(1));
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    mc.bbox.min =
        p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(p1.col(1));
    mc.bbox.max =
        p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(p1.col(1));
    mc.bbox.pad_inplace(bbox_padding);
    mesh_colliders.emplace_back(std::move(mc));
  }
  // Create triangle colliders for continuous collision detection.
  for (int i = 0; i < face_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;
    mc.type = MeshColliderType::Triangle;
    mc.index = m.F.row(i);
    mc.inv_mass(0) = get_inv_mass(mc.index(0));
    mc.inv_mass(1) = get_inv_mass(mc.index(1));
    mc.inv_mass(2) = get_inv_mass(mc.index(2));
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p0.col(2) = m.V.row(mc.index(2));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    p1.col(2) = m.V.row(mc.index(2));
    mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
    mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
    mc.bbox.pad_inplace(bbox_padding);
    mesh_colliders.emplace_back(std::move(mc));
  }
  return mesh_colliders;
}

ObjectCollider::ObjectCollider(Handle entity_handle,
                               const CollisionConfig& config,
                               const TriMesh& mesh, const Pin& pin,
                               const Eigen::VectorXf& mass, int state_offset) {
  auto& c = config;
  this->entity_handle = entity_handle;
  this->state_offset = state_offset;
  // Pad object AABB by 5% of mesh scale to avoid zero-width degenerate cases.
  bbox_padding = 0.05f * mesh.avg_edge_length;
  Eigen::Vector3f min = mesh.V.colwise().minCoeff();
  Eigen::Vector3f max = mesh.V.colwise().maxCoeff();
  bbox = Bbox{min, max};
  bbox.pad_inplace(bbox_padding);
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
  mesh_collider_tree.init(
      make_mesh_colliders(mesh, bbox_padding, get_inv_mass));
}

ObjectCollider::ObjectCollider(Handle entity_handle,
                               const CollisionConfig& config,
                               const TriMesh& mesh) {
  auto& c = config;
  this->entity_handle = entity_handle;
  state_offset = -1;
  // Pad object AABB by 5% of mesh scale to avoid zero-width degenerate cases.
  bbox_padding = 0.05f * mesh.avg_edge_length;
  Eigen::Vector3f min = mesh.V.colwise().minCoeff();
  Eigen::Vector3f max = mesh.V.colwise().maxCoeff();
  bbox = Bbox{min, max};
  bbox.pad_inplace(bbox_padding);
  // Note: group == -1 disables this collider in the broad phase.
  group = (c.is_collision_on) ? c.group : -1;
  is_static = false;
  is_self_collision_on = c.is_self_collision_on;
  restitution = config.restitution;
  friction = config.friction;
  // Obstacles have infinite mass (zero inverse mass).
  auto get_inv_mass = [](int) { return 0.0f; };
  mesh_collider_tree.init(
      make_mesh_colliders(mesh, bbox_padding, get_inv_mass));
}

void ObjectCollider::update(const CollisionConfig& config,
                            const ObjectState& object_state,
                            const Eigen::VectorXf global_curr_state,
                            const Eigen::VectorXf global_prev_state) {
  auto& c = config;
  group = (c.is_collision_on) ? c.group : -1;
  is_self_collision_on = c.is_self_collision_on;
  restitution = config.restitution;
  friction = config.friction;
  // Extract vertex positions from state vector.
  auto get_vertex = [](Eigen::Ref<const Eigen::VectorXf> vec,
                       int index) -> Eigen::Vector3f {
    return vec(Eigen::seqN(3 * index, 3));
  };
  auto seq = Eigen::seqN(object_state.state_offset, object_state.state_num);
  auto curr_state = global_curr_state(seq);
  auto prev_state = global_prev_state(seq);
  bbox = Bbox{curr_state(Eigen::seqN(0, 3)), curr_state(Eigen::seqN(0, 3))};
  auto& colliders = mesh_collider_tree.get_colliders();
  int collider_num = colliders.size();
  tbb::enumerable_thread_specific<Bbox> thread_bboxes(bbox);
  tbb::parallel_for(0, collider_num, [&](int i) {
    MeshCollider& mc = colliders[i];
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;
    Bbox& bbox = thread_bboxes.local();
    switch (mc.type) {
      case MeshColliderType::Point: {
        p0.col(0) = get_vertex(prev_state, mc.index(0));
        p1.col(0) = get_vertex(curr_state, mc.index(0));
        // Bounding box must contain trajectory between timesteps.
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        p0.col(0) = get_vertex(prev_state, mc.index(0));
        p0.col(1) = get_vertex(prev_state, mc.index(1));
        p1.col(0) = get_vertex(curr_state, mc.index(0));
        p1.col(1) = get_vertex(curr_state, mc.index(1));
        // Bounding box contains swept volume of edge between timesteps.
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(
                p1.col(1));
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Triangle: {
        p0.col(0) = get_vertex(prev_state, mc.index(0));
        p0.col(1) = get_vertex(prev_state, mc.index(1));
        p0.col(2) = get_vertex(prev_state, mc.index(2));
        p1.col(0) = get_vertex(curr_state, mc.index(0));
        p1.col(1) = get_vertex(curr_state, mc.index(1));
        p1.col(2) = get_vertex(curr_state, mc.index(2));
        // Bounding box contains swept volume of triangle between timesteps.
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  });
  // Merge thread-local bounding boxes into final object bounding box.
  for (auto& tbbox : thread_bboxes) {
    bbox.merge_inplace(tbbox);
  }
  mesh_collider_tree.update(bbox);
}

void ObjectCollider::update(const CollisionConfig& config,
                            const ObstaclePosition& obstacle_position) {
  const CollisionConfig& c = config;
  const ObstaclePosition& p = obstacle_position;
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
  auto& position_t1 = p.curr_position;
  // Extract vertex positions from state vector.
  auto get_vertex = [](Eigen::Ref<const Eigen::VectorXf> vec,
                       int index) -> Eigen::Vector3f {
    return vec(Eigen::seqN(3 * index, 3));
  };
  bbox = Bbox{p.curr_position(Eigen::seqN(0, 3)),
              p.curr_position(Eigen::seqN(0, 3))};
  auto& colliders = mesh_collider_tree.get_colliders();
  int collider_num = colliders.size();
  tbb::enumerable_thread_specific<Bbox> thread_bboxes(bbox);
  tbb::parallel_for(0, collider_num, [&](int i) {
    MeshCollider& mc = colliders[i];
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;
    Bbox& bbox = thread_bboxes.local();
    switch (mc.type) {
      case MeshColliderType::Point: {
        p0.col(0) = get_vertex(position_t0, mc.index(0));
        p1.col(0) = get_vertex(position_t1, mc.index(0));
        // Bounding box must contain trajectory between timesteps.
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        p0.col(0) = get_vertex(position_t0, mc.index(0));
        p0.col(1) = get_vertex(position_t0, mc.index(1));
        p1.col(0) = get_vertex(position_t1, mc.index(0));
        p1.col(1) = get_vertex(position_t1, mc.index(1));
        // Bounding box contains swept volume of edge between timesteps.
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(
                p1.col(1));
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Triangle: {
        p0.col(0) = get_vertex(position_t0, mc.index(0));
        p0.col(1) = get_vertex(position_t0, mc.index(1));
        p0.col(2) = get_vertex(position_t0, mc.index(2));
        p1.col(0) = get_vertex(position_t1, mc.index(0));
        p1.col(1) = get_vertex(position_t1, mc.index(1));
        p1.col(2) = get_vertex(position_t1, mc.index(2));
        // Bounding box contains swept volume of triangle between timesteps.
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  });
  // Merge thread-local bounding boxes into final object bounding box.
  for (auto& tbbox : thread_bboxes) {
    bbox.merge_inplace(tbbox);
  }
  mesh_collider_tree.update(bbox);
}

}  // namespace silk::cpu
