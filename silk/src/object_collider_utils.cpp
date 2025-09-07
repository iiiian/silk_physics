#include "object_collider_utils.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <functional>
#include <unordered_set>

#include "cloth_solver_data.hpp"
#include "handle.hpp"
#include "mesh.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver_state.hpp"

namespace silk {

// Create vertex, edge, and face mesh colliders.
std::vector<MeshCollider> make_mesh_colliders(
    const TriMesh& mesh, float bbox_padding,
    std::function<float(int)> get_inv_mass) {
  auto& m = mesh;

  int vert_num = m.V.rows();
  int edge_num = m.E.rows();
  int face_num = m.F.rows();

  std::vector<MeshCollider> mesh_colliders;

  // Create vertex collider
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

  // Create edge collider
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
    mc.bbox.min = p0.col(0).cwiseMin(p0.col(1));
    mc.bbox.max = p0.col(0).cwiseMax(p0.col(1));
    mc.bbox.pad_inplace(bbox_padding);

    mesh_colliders.emplace_back(std::move(mc));
  }

  // Create face collider
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

ObjectCollider make_physical_object_collider(
    Handle entity_handle, const CollisionConfig& config, const TriMesh& mesh,
    const Pin& pin, const Eigen::VectorXf& mass, int state_offset) {
  auto& c = config;

  ObjectCollider o;
  o.entity_handle = entity_handle;
  o.state_offset = state_offset;

  // Heuristic: pad object AABB by 5% of mesh scale to avoid zero width bbox.
  o.bbox_padding = 0.05f * mesh.avg_edge_length;
  Eigen::Vector3f min = mesh.V.colwise().minCoeff();
  Eigen::Vector3f max = mesh.V.colwise().maxCoeff();
  o.bbox = Bbox{min, max};
  o.bbox.pad_inplace(o.bbox_padding);

  // Note: group == -1 disables this collider in the broad phase.
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.is_obstacle = false;
  o.is_self_collision_on = c.is_self_collision_on;
  o.restitution = config.restitution;
  o.friction = config.friction;

  std::unordered_set<int> pin_set(pin.index.begin(), pin.index.end());
  // inv mass = 0 if vertex is pinned.
  auto get_inv_mass = [&pin_set, &mass](int index) -> float {
    if (pin_set.find(index) != pin_set.end()) {
      return 0.0f;
    }
    return 1.0f / mass(index);
  };

  o.mesh_collider_tree.init(
      make_mesh_colliders(mesh, o.bbox_padding, get_inv_mass));

  return o;
}

ObjectCollider make_obstacle_object_collider(Handle entity_handle,
                                             const CollisionConfig& config,
                                             const TriMesh& mesh) {
  auto& c = config;

  ObjectCollider o;
  o.entity_handle = entity_handle;
  o.state_offset = -1;

  // Heuristic: pad object AABB by 5% of mesh scale to avoid zero width bbox.
  o.bbox_padding = 0.05f * mesh.avg_edge_length;
  Eigen::Vector3f min = mesh.V.colwise().minCoeff();
  Eigen::Vector3f max = mesh.V.colwise().maxCoeff();
  o.bbox = Bbox{min, max};
  o.bbox.pad_inplace(o.bbox_padding);

  // Note: group == -1 disables this collider in the broad phase.
  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = false;
  o.is_obstacle = true;
  o.is_self_collision_on = c.is_self_collision_on;
  o.restitution = config.restitution;
  o.friction = config.friction;

  // for obstacle, inv mass is always 0.
  auto get_inv_mass = [](int index) { return 0.0f; };
  o.mesh_collider_tree.init(
      make_mesh_colliders(mesh, o.bbox_padding, get_inv_mass));

  return o;
}

void make_all_object_collider(Registry& registry) {
  // Lazily attach colliders to entities that have enough data. Existing
  // colliders are preserved to avoid unnecessary rebuilds.
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<CollisionConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto pin = registry.get<Pin>(e);
    auto cloth_static_data = registry.get<ClothStaticSolverData>(e);
    auto solver_state = registry.get<SolverState>(e);
    auto object_collider = registry.get<ObjectCollider>(e);

    if (object_collider) {
      continue;
    }

    if (config && mesh && pin && cloth_static_data && solver_state) {
      registry.set<ObjectCollider>(
          e, make_physical_object_collider(e.self, *config, *mesh, *pin,
                                           cloth_static_data->mass,
                                           solver_state->state_offset));
      continue;
    };

    if (config && mesh) {
      registry.set<ObjectCollider>(
          e, make_obstacle_object_collider(e.self, *config, *mesh));
      continue;
    }
  }
}

void update_physical_object_collider(
    const CollisionConfig& config, Eigen::Ref<const Eigen::VectorXf> curr_state,
    Eigen::Ref<const Eigen::VectorXf> prev_state,
    ObjectCollider& object_collider) {
  auto& o = object_collider;
  auto& c = config;

  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_self_collision_on = c.is_self_collision_on;
  o.restitution = config.restitution;
  o.friction = config.friction;

  // Always update mesh colliders for physical entity.

  auto get_vertex = [](Eigen::Ref<const Eigen::VectorXf> vec,
                       int index) -> Eigen::Vector3f {
    return vec(Eigen::seqN(3 * index, 3));
  };

  // Init bbox with the position of the first vertex.
  o.bbox = Bbox{curr_state(Eigen::seqN(0, 3)), curr_state(Eigen::seqN(0, 3))};
  std::vector<Bbox> thread_local_bboxes(omp_get_max_threads(), o.bbox);

  auto& colliders = o.mesh_collider_tree.get_colliders();
#pragma omp parallel for
  for (int i = 0; i < colliders.size(); ++i) {
    MeshCollider& mc = colliders[i];
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    Bbox& bbox = thread_local_bboxes[omp_get_thread_num()];

    switch (mc.type) {
      case MeshColliderType::Point: {
        p0.col(0) = get_vertex(prev_state, mc.index(0));
        p1.col(0) = get_vertex(curr_state, mc.index(0));
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(o.bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        p0.col(0) = get_vertex(prev_state, mc.index(0));
        p0.col(1) = get_vertex(prev_state, mc.index(1));
        p1.col(0) = get_vertex(curr_state, mc.index(0));
        p1.col(1) = get_vertex(curr_state, mc.index(1));
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(
                p1.col(1));
        mc.bbox.pad_inplace(o.bbox_padding);
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
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(o.bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  }

  for (auto& bbox : thread_local_bboxes) {
    o.bbox.merge_inplace(bbox);
  }

  o.mesh_collider_tree.update(o.bbox);
}

void update_all_physical_object_collider(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& prev_global_state) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<CollisionConfig>(e);
    auto state = registry.get<SolverState>(e);
    auto collider = registry.get<ObjectCollider>(e);

    if (config && state && collider) {
      auto seq = Eigen::seqN(state->state_offset, state->state_num);
      update_physical_object_collider(*config, global_state(seq),
                                      prev_global_state(seq), *collider);
      continue;
    };
  }
}

void update_obstacle_object_collider(const CollisionConfig& config,
                                     const ObstaclePosition& obstacle_position,
                                     ObjectCollider& object_collider) {
  const CollisionConfig& c = config;
  const ObstaclePosition& p = obstacle_position;
  ObjectCollider& o = object_collider;

  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = p.is_static;
  o.restitution = config.restitution;
  o.friction = config.friction;

  // If the obstacle is static, skip updates.
  if (o.is_static) {
    return;
  }

  auto get_vertex = [](Eigen::Ref<const Eigen::VectorXf> vec,
                       int index) -> Eigen::Vector3f {
    return vec(Eigen::seqN(3 * index, 3));
  };

  // Init bbox with the position of the first vertex.
  o.bbox = Bbox{p.position(Eigen::seqN(0, 3)), p.position(Eigen::seqN(0, 3))};
  std::vector<Bbox> thread_local_bboxes(omp_get_max_threads(), o.bbox);

  auto& colliders = o.mesh_collider_tree.get_colliders();

#pragma omp parallel for
  for (int i = 0; i < colliders.size(); ++i) {
    MeshCollider& mc = colliders[i];
    auto& p0 = mc.position_t0;
    auto& p1 = mc.position_t1;

    Bbox& bbox = thread_local_bboxes[omp_get_thread_num()];

    switch (mc.type) {
      case MeshColliderType::Point: {
        p0.col(0) = get_vertex(p.prev_position, mc.index(0));
        p1.col(0) = get_vertex(p.position, mc.index(0));
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(o.bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        p0.col(0) = get_vertex(p.prev_position, mc.index(0));
        p0.col(1) = get_vertex(p.prev_position, mc.index(1));
        p1.col(0) = get_vertex(p.position, mc.index(0));
        p1.col(1) = get_vertex(p.position, mc.index(1));
        mc.bbox.min =
            p0.col(0).cwiseMin(p0.col(1)).cwiseMin(p1.col(0)).cwiseMin(
                p1.col(1));
        mc.bbox.max =
            p0.col(0).cwiseMax(p0.col(1)).cwiseMax(p1.col(0)).cwiseMax(
                p1.col(1));
        mc.bbox.pad_inplace(o.bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Triangle: {
        p0.col(0) = get_vertex(p.prev_position, mc.index(0));
        p0.col(1) = get_vertex(p.prev_position, mc.index(1));
        p0.col(2) = get_vertex(p.prev_position, mc.index(2));
        p1.col(0) = get_vertex(p.position, mc.index(0));
        p1.col(1) = get_vertex(p.position, mc.index(1));
        p1.col(2) = get_vertex(p.position, mc.index(2));
        mc.bbox.min = p0.rowwise().minCoeff().cwiseMin(p1.rowwise().minCoeff());
        mc.bbox.max = p0.rowwise().maxCoeff().cwiseMax(p1.rowwise().maxCoeff());
        mc.bbox.pad_inplace(o.bbox_padding);
        bbox.merge_inplace(mc.bbox);
        break;
      }
    }
  }

  for (auto& bbox : thread_local_bboxes) {
    o.bbox.merge_inplace(bbox);
  }

  o.mesh_collider_tree.update(o.bbox);
}

void update_all_obstacle_object_collider(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<CollisionConfig>(e);
    auto position = registry.get<ObstaclePosition>(e);
    auto collider = registry.get<ObjectCollider>(e);

    if (config && position && collider) {
      update_obstacle_object_collider(*config, *position, *collider);
      continue;
    };
  }
}

}  // namespace silk
