#include "make_obstacle.hpp"

#include <Eigen/Core>
#include <unordered_set>

#include "mesh.hpp"
#include "pin_group.hpp"
#include "silk/silk.hpp"
#include "solver_data.hpp"

namespace silk {

// for physical object
Obstacle make_obstacle(const CollisionConfig& config, const TriMesh& tri_mesh,
                       const PinGroup& pin_group,
                       const SolverData& solver_data) {
  const CollisionConfig& c = config;
  const TriMesh& m = tri_mesh;
  const Eigen::VectorXf& mass = solver_data.mass;

  std::unordered_set<int> pin_set(pin_group.pinnned_index.begin(),
                                  pin_group.pinnned_index.end());
  auto is_pinned = [&pin_set](int index) -> bool {
    return (pin_set.find(index) != pin_set.end());
  };

  Obstacle o;
  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  // group -1 means collision is disabled
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.solver_offset = solver_data.state_offset;
  o.is_self_collision_on = c.is_self_collision_on;
  o.bbox_padding = 0.05f * m.avg_edge_length;
  o.damping = config.damping;
  o.friction = config.friction;

  int vert_num = m.V.rows();
  int edge_num = m.E.rows();
  int face_num = m.F.rows();

  // create mesh collider for every vertex
  for (int i = 0; i < vert_num; ++i) {
    MeshCollider mc;
    mc.type = MeshColliderType::Point;
    mc.index(0) = i;
    mc.weight(0) = is_pinned(i) ? 0.0f : mass(i);
    mc.position_t0.col(0) = m.V.row(i);
    mc.position_t1.col(1) = m.V.row(i);
    mc.bbox.min = m.V.row(i);
    mc.bbox.max = m.V.row(i);
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  // create mesh collider for every edge
  for (int i = 0; i < edge_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;
    mc.type = MeshColliderType::Edge;
    mc.index = m.E.row(i);
    mc.weight(0) = is_pinned(mc.index(0)) ? 0.0f : mass(mc.index(0));
    mc.weight(1) = is_pinned(mc.index(1)) ? 0.0f : mass(mc.index(1));
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    mc.bbox.min = p0.col(0).cwiseMin(p0.col(1));
    mc.bbox.max = p0.col(0).cwiseMax(p0.col(1));
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  // create mesh collider for every face
  for (int i = 0; i < face_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    mc.type = MeshColliderType::Triangle;
    mc.index = m.F.row(i);
    mc.weight(0) = is_pinned(mc.index(0)) ? 0.0f : mass(mc.index(0));
    mc.weight(1) = is_pinned(mc.index(1)) ? 0.0f : mass(mc.index(1));
    mc.weight(2) = is_pinned(mc.index(2)) ? 0.0f : mass(mc.index(2));
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p0.col(2) = m.V.row(mc.index(2));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    p1.col(2) = m.V.row(mc.index(2));
    mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
    mc.bbox.min = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  o.mesh_collider_tree.init(o.mesh_colliders.data(), o.mesh_colliders.size());

  return o;
}

// for pure obstacle
Obstacle make_obstacle(const CollisionConfig& config, const TriMesh& tri_mesh) {
  const CollisionConfig& c = config;
  const TriMesh& m = tri_mesh;

  Obstacle o;
  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  // group -1 means collision is disabled
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.solver_offset = -1;
  o.is_self_collision_on = c.is_self_collision_on;
  o.bbox_padding = 0.05f * m.avg_edge_length;
  o.damping = config.damping;
  o.friction = config.friction;

  int vert_num = m.V.rows();
  int edge_num = m.E.rows();
  int face_num = m.F.rows();

  // create mesh collider for every vertex
  for (int i = 0; i < vert_num; ++i) {
    MeshCollider mc;
    mc.type = MeshColliderType::Point;
    mc.index(0) = i;
    mc.weight(0) = 0.0f;
    mc.position_t0.col(0) = m.V.row(i);
    mc.position_t1.col(1) = m.V.row(i);
    mc.bbox.min = m.V.row(i);
    mc.bbox.max = m.V.row(i);
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  // create mesh collider for every edge
  for (int i = 0; i < edge_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    mc.type = MeshColliderType::Edge;
    mc.index = m.E.row(i);
    mc.weight(0) = 0.0f;
    mc.weight(1) = 0.0f;
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    mc.bbox.min = p0.col(0).cwiseMin(p0.col(1));
    mc.bbox.max = p0.col(0).cwiseMax(p0.col(1));
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  // create mesh collider for every face
  for (int i = 0; i < face_num; ++i) {
    MeshCollider mc;
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    mc.type = MeshColliderType::Triangle;
    mc.index = m.F.row(i);
    mc.weight(0) = 0.0f;
    mc.weight(1) = 0.0f;
    mc.weight(2) = 0.0f;
    p0.col(0) = m.V.row(mc.index(0));
    p0.col(1) = m.V.row(mc.index(1));
    p0.col(2) = m.V.row(mc.index(2));
    p1.col(0) = m.V.row(mc.index(0));
    p1.col(1) = m.V.row(mc.index(1));
    p1.col(2) = m.V.row(mc.index(2));
    mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
    mc.bbox.min = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
    mc.bbox.pad_inplace(o.bbox_padding);

    o.bbox.merge_inplace(mc.bbox);
    o.mesh_colliders.emplace_back(std::move(mc));
  }

  o.mesh_collider_tree.init(o.mesh_colliders.data(), o.mesh_colliders.size());

  return o;
}

void make_all_obstacles(Registry& registry) {
  for (Entity& e : registry.entity.data()) {
    // remove old obstacle if exists
    ECS_REMOVE(registry, e, obstacle);

    auto collision_config = ECS_GET_PTR(registry, e, collision_config);
    auto tri_mesh = ECS_GET_PTR(registry, e, tri_mesh);
    auto pin_group = ECS_GET_PTR(registry, e, pin_group);
    auto solver_data = ECS_GET_PTR(registry, e, solver_data);

    Handle h;
    if (tri_mesh && pin_group && solver_data) {
      h = registry.obstacle.add(make_obstacle(*collision_config, *tri_mesh,
                                              *pin_group, *solver_data));
      assert(!h.is_empty());
      e.obstacle = h;
      continue;
    };

    if (tri_mesh) {
      h = registry.obstacle.add(make_obstacle(*collision_config, *tri_mesh));
      assert(!h.is_empty());
      e.obstacle = h;
      continue;
    }

    // add obstacle to registry and link to entity
  }
}

}  // namespace silk
