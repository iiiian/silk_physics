#include "object_collider_utils.hpp"

#include <Eigen/Core>
#include <unordered_set>

#include "mesh.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver_data.hpp"

namespace silk {

ObjectCollider make_physical_object_collider(const CollisionConfig& config,
                                             const TriMesh& tri_mesh,
                                             const Pin& pin,
                                             const SolverData& solver_data) {
  const CollisionConfig& c = config;
  const TriMesh& m = tri_mesh;
  const Eigen::VectorXf& mass = solver_data.mass;

  std::unordered_set<int> pin_set(pin.index.begin(), pin.index.end());
  auto is_pinned = [&pin_set](int index) -> bool {
    return (pin_set.find(index) != pin_set.end());
  };

  ObjectCollider o;
  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  // group -1 means collision is disabled
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.solver_offset = solver_data.state_offset;
  o.is_self_collision_on = c.is_self_collision_on;
  // o.bbox_padding = 0.05f * m.avg_edge_length;
  o.bbox_padding = 1e-6f;
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
    mc.index(Eigen::seqN(0, 2)) = m.E.row(i);
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

ObjectCollider make_obstacle_object_collider(const CollisionConfig& config,
                                             const TriMesh& tri_mesh) {
  const CollisionConfig& c = config;
  const TriMesh& m = tri_mesh;

  ObjectCollider o;
  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  // group -1 means collision is disabled
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.solver_offset = -1;
  o.is_self_collision_on = c.is_self_collision_on;
  // o.bbox_padding = 0.05f * m.avg_edge_length;
  o.bbox_padding = 1e-6f;
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
    mc.index(Eigen::seqN(0, 2)) = m.E.row(i);
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

void init_all_object_collider(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto collision_config = registry.get<CollisionConfig>(e);
    auto tri_mesh = registry.get<TriMesh>(e);
    auto pin = registry.get<Pin>(e);
    auto solver_data = registry.get<SolverData>(e);
    auto object_collider = registry.get<ObjectCollider>(e);

    if (object_collider) {
      continue;
    }

    if (collision_config && tri_mesh && pin && solver_data) {
      registry.set<ObjectCollider>(
          e, make_physical_object_collider(*collision_config, *tri_mesh, *pin,
                                           *solver_data));
      continue;
    };

    if (collision_config && tri_mesh) {
      registry.set<ObjectCollider>(
          e, make_obstacle_object_collider(*collision_config, *tri_mesh));
      continue;
    }
  }
}

void update_physical_object_collider(const CollisionConfig& config,
                                     const SolverData& solver_data,
                                     const Eigen::VectorXf& solver_state,
                                     const Eigen::VectorXf& prev_solver_state,
                                     ObjectCollider& object_collider) {
  ObjectCollider& o = object_collider;
  const CollisionConfig& c = config;

  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  o.group = (c.is_collision_on) ? c.group : -1;
  o.solver_offset = solver_data.state_offset;
  o.is_self_collision_on = c.is_self_collision_on;
  o.damping = config.damping;
  o.friction = config.friction;

  // always update mesh colliders for dynamic entity

  auto get_vertex = [](const Eigen::VectorXf& vec,
                       int offset) -> Eigen::Vector3f {
    return vec(Eigen::seqN(offset, 3));
  };

  for (MeshCollider& mc : o.mesh_colliders) {
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    switch (mc.type) {
      case MeshColliderType::Point: {
        int o0 = solver_data.state_offset + 3 * mc.index(0);
        p0.col(0) = get_vertex(prev_solver_state, o0);
        p1.col(0) = get_vertex(solver_state, o0);
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        int o0 = solver_data.state_offset + 3 * mc.index(0);
        int o1 = solver_data.state_offset + 3 * mc.index(1);
        p0.col(0) = get_vertex(prev_solver_state, o0);
        p0.col(1) = get_vertex(prev_solver_state, o1);
        p1.col(0) = get_vertex(solver_state, o0);
        p1.col(1) = get_vertex(solver_state, o1);
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Triangle: {
        int o0 = solver_data.state_offset + 3 * mc.index(0);
        int o1 = solver_data.state_offset + 3 * mc.index(1);
        int o2 = solver_data.state_offset + 3 * mc.index(2);
        p0.col(0) = get_vertex(prev_solver_state, o0);
        p0.col(1) = get_vertex(prev_solver_state, o1);
        p0.col(2) = get_vertex(prev_solver_state, o2);
        p1.col(0) = get_vertex(solver_state, o0);
        p1.col(1) = get_vertex(solver_state, o1);
        p1.col(2) = get_vertex(solver_state, o2);
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  }

  o.mesh_collider_tree.update(o.bbox);
}

void update_all_physical_object_collider(
    Registry& registry, const Eigen::VectorXf& solver_state,
    const Eigen::VectorXf& prev_solver_state) {
  for (Entity& e : registry.get_all_entities()) {
    auto collision_config = registry.get<CollisionConfig>(e);
    auto tri_mesh = registry.get<TriMesh>(e);
    auto pin = registry.get<Pin>(e);
    auto solver_data = registry.get<SolverData>(e);
    auto object_collider = registry.get<ObjectCollider>(e);

    if (collision_config && tri_mesh && pin && solver_data && object_collider) {
      update_physical_object_collider(*collision_config, *solver_data,
                                      solver_state, prev_solver_state,
                                      *object_collider);
      continue;
    };
  }
}

void update_obstacle_object_collider(const CollisionConfig& config,
                                     const ObstaclePosition& position,
                                     ObjectCollider& object_collider) {
  const CollisionConfig& c = config;
  const ObstaclePosition& p = position;
  ObjectCollider& o = object_collider;

  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = p.is_static;

  // if pure collider is static, do not update mesh colliders
  if (o.is_static) {
    return;
  }

  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};

  auto get_vertex = [](const Eigen::VectorXf& vec,
                       int offset) -> Eigen::Vector3f {
    return vec(Eigen::seqN(offset, 3));
  };

  for (MeshCollider& mc : o.mesh_colliders) {
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    switch (mc.type) {
      case MeshColliderType::Point: {
        int o0 = 3 * mc.index(0);
        p0.col(0) = get_vertex(p.prev_position, o0);
        p1.col(0) = get_vertex(p.position, o0);
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        int o0 = 3 * mc.index(0);
        int o1 = 3 * mc.index(1);
        p0.col(0) = get_vertex(p.prev_position, o0);
        p0.col(1) = get_vertex(p.prev_position, o1);
        p1.col(0) = get_vertex(p.position, o0);
        p1.col(1) = get_vertex(p.position, o1);
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(
                p1.col(1));
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Triangle: {
        int o0 = 3 * mc.index(0);
        int o1 = 3 * mc.index(1);
        int o2 = 3 * mc.index(2);
        p0.col(0) = get_vertex(p.prev_position, o0);
        p0.col(1) = get_vertex(p.prev_position, o1);
        p0.col(2) = get_vertex(p.prev_position, o2);
        p1.col(0) = get_vertex(p.position, o0);
        p1.col(1) = get_vertex(p.position, o1);
        p1.col(2) = get_vertex(p.position, o2);
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  }

  o.mesh_collider_tree.update(o.bbox);
}

void update_all_obstacle_object_collider(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto collision_config = registry.get<CollisionConfig>(e);
    auto obstacle_position = registry.get<ObstaclePosition>(e);
    auto object_collider = registry.get<ObjectCollider>(e);

    if (collision_config && obstacle_position && object_collider) {
      update_obstacle_object_collider(*collision_config, *obstacle_position,
                                      *object_collider);
      continue;
    };
  }
}

}  // namespace silk
