#include "update_obstacle.hpp"

#include <Eigen/Core>

#include "obstacle.hpp"
#include "pin_group.hpp"
#include "silk/silk.hpp"
#include "solver_data.hpp"

namespace silk {

void update_obstacle(const CollisionConfig& config,
                     const SolverData& solver_data,
                     const Eigen::VectorXf& solver_state,
                     const Eigen::VectorXf& prev_solver_state,
                     Obstacle& obstacle) {
  Obstacle& o = obstacle;
  const CollisionConfig& c = config;

  o.bbox = Bbox{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  o.group = (c.is_collision_on) ? c.group : -1;
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

void update_obstacle(const CollisionConfig& config, const PinGroup& pin_group,
                     Obstacle& obstacle) {
  Obstacle& o = obstacle;
  const CollisionConfig& c = config;

  o.group = (c.is_collision_on) ? c.group : -1;
  o.is_static = (pin_group.prev_pin_value.size() == 0);

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
        p0.col(0) = get_vertex(pin_group.prev_pin_value, o0);
        p1.col(0) = get_vertex(pin_group.pin_value, o0);
        mc.bbox.min = p0.col(0).cwiseMin(p1.col(0));
        mc.bbox.max = p0.col(0).cwiseMax(p1.col(0));
        mc.bbox.pad_inplace(o.bbox_padding);
        o.bbox.merge_inplace(mc.bbox);
        break;
      }
      case MeshColliderType::Edge: {
        int o0 = 3 * mc.index(0);
        int o1 = 3 * mc.index(1);
        p0.col(0) = get_vertex(pin_group.prev_pin_value, o0);
        p0.col(1) = get_vertex(pin_group.prev_pin_value, o1);
        p1.col(0) = get_vertex(pin_group.pin_value, o0);
        p1.col(1) = get_vertex(pin_group.pin_value, o1);
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
        p0.col(0) = get_vertex(pin_group.prev_pin_value, o0);
        p0.col(1) = get_vertex(pin_group.prev_pin_value, o1);
        p0.col(2) = get_vertex(pin_group.prev_pin_value, o2);
        p1.col(0) = get_vertex(pin_group.pin_value, o0);
        p1.col(1) = get_vertex(pin_group.pin_value, o1);
        p1.col(2) = get_vertex(pin_group.pin_value, o2);
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

void update_all_obstacles(Registry& registry,
                          const Eigen::VectorXf& solver_state,
                          const Eigen::VectorXf& prev_solver_state) {
  for (const Entity& e : registry.entity.data()) {
    auto collision_config = ECS_GET_PTR(registry, e, collision_config);
    auto pin_group = ECS_GET_PTR(registry, e, pin_group);
    auto solver_data = ECS_GET_PTR(registry, e, solver_data);
    auto obstacle = ECS_GET_PTR(registry, e, obstacle);

    if (collision_config && solver_data && obstacle) {
      update_obstacle(*collision_config, *solver_data, solver_state,
                      prev_solver_state, *obstacle);
      continue;
    }

    if (collision_config && pin_group && obstacle) {
      update_obstacle(*collision_config, *pin_group, *obstacle);
      continue;
    }
  }
}

}  // namespace silk
