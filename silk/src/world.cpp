#include <igl/edges.h>

#include <Eigen/Core>
#include <cassert>
#include <cstring>
#include <silk/result.hpp>
#include <silk/silk.hpp>

#include "cloth_solver_data.hpp"
#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "mesh_utils.hpp"
#include "object_state.hpp"
#include "solver/solver_pipeline.hpp"

namespace silk {

class World::WorldImpl {
 private:
  Registry registry_;
  SolverPipeline solver_pipeline_;
  CollisionPipeline collision_pipeline_;

 public:
  // Global API
  Result set_global_config(GlobalConfig config) {
    auto& c = config;
    solver_pipeline_.const_acceleration = {c.acceleration_x, c.acceleration_y,
                                           c.acceleration_z};
    // solver_.max_inner_iteration = c.max_iteration;
    solver_pipeline_.dt = c.dt;
    solver_pipeline_.max_outer_iteration = c.max_outer_iteration;
    solver_pipeline_.max_inner_iteration = c.max_inner_iteration;

    return Result::ok();
  }

  void clear() {
    solver_pipeline_.clear(registry_);
    registry_.clear();
  }

  // Solver API

  Result solver_step() {
    if (!solver_pipeline_.step(registry_, collision_pipeline_)) {
      return Result::error(ErrorCode::CholeskyDecompositionFail);
    }

    for (Entity& e : registry_.get_all_entities()) {
      auto obstacle_position = registry_.get<ObstaclePosition>(e);
      if (!obstacle_position) {
        continue;
      }

      if (obstacle_position->position.size() != 0) {
        std::swap(obstacle_position->position,
                  obstacle_position->prev_position);
        obstacle_position->is_static = true;
      }
    }

    return Result::ok();
  }

  Result solver_reset() {
    solver_pipeline_.reset(registry_);
    return Result::ok();
  }

  // cloth API
  Result add_cloth(ClothConfig cloth_config, CollisionConfig collision_config,
                   MeshConfig mesh_config, ConstSpan<int> pin_index,
                   uint32_t& handle) {
    // make tri mesh
    auto tri_mesh = try_make_cloth_mesh(mesh_config);
    if (!tri_mesh) {
      return Result::error(ErrorCode::InvalidMesh);
    }

    // make pin
    Pin p;
    if (pin_index.data != nullptr && pin_index.size != 0) {
      p.index =
          Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.size);
      p.position.resize(3 * p.index.size());
      for (int i = 0; i < p.index.size(); ++i) {
        p.position(Eigen::seqN(3 * i, 3)) = tri_mesh->V.row(p.index(i));
      }
    }

    auto [h, e] = registry_.add_entity();
    if (h.is_empty()) {
      return Result::error(ErrorCode::TooManyBody);
    }
    assert(e);
    registry_.set<ClothConfig>(*e, std::move(cloth_config));
    registry_.set<CollisionConfig>(*e, std::move(collision_config));
    registry_.set<TriMesh>(*e, std::move(*tri_mesh));
    registry_.set<Pin>(*e, std::move(p));

    handle = h.value;
    return Result::ok();
  }

  Result remove_cloth(uint32_t handle) {
    auto entity = registry_.get_entity(handle);
    if (!entity) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto cloth_config = registry_.get<ClothConfig>(*entity);
    if (!cloth_config) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    registry_.remove_entity(handle);

    return Result::ok();
  }

  Result get_cloth_position(uint32_t handle, Span<float> position) const {
    const Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto solver_state = registry_.get<ObjectState>(e);
    if (solver_state) {
      if (position.size < solver_state->state_num) {
        return Result::error(ErrorCode::IncorrectPositionNum);
      }

      memcpy(position.data, solver_state->curr_state.data(),
             solver_state->state_num * sizeof(float));
      return Result::ok();
    }

    auto mesh = registry_.get<TriMesh>(e);
    if (mesh) {
      int state_num = 3 * mesh->V.rows();
      if (position.size != state_num) {
        return Result::error(ErrorCode::IncorrectPositionNum);
      }

      memcpy(position.data, mesh->V.data(), state_num * sizeof(float));
      return Result::ok();
    }

    return Result::error(ErrorCode::InvalidHandle);
  }

  Result set_cloth_config(uint32_t handle, ClothConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    *cloth_config = config;

    // remove outdated components
    registry_.remove<ClothSolverContext>(*e);
    registry_.remove<ObjectCollider>(*e);

    return Result::ok();
  }

  Result set_cloth_collision_config(uint32_t handle, CollisionConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto collision_config = registry_.get<CollisionConfig>(*e);
    assert(collision_config);

    *collision_config = config;
    return Result::ok();
  }

  Result set_cloth_pin_index(uint32_t handle, ConstSpan<int> pin_index) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    // make tri mesh
    auto tri_mesh = registry_.get<TriMesh>(*e);
    assert(tri_mesh);

    // make pin
    auto pin = registry_.get<Pin>(*e);
    assert(pin);
    if (pin_index.data != nullptr && pin_index.size != 0) {
      pin->index =
          Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.size);
      pin->position.resize(3 * pin->index.size());
      for (int i = 0; i < pin->index.size(); ++i) {
        pin->position(Eigen::seqN(3 * i, 3)) = tri_mesh->V.row(pin->index(i));
      }
    } else {
      pin->index = {};
      pin->position = {};
    }

    // remove outdated components
    registry_.remove<ClothSolverContext>(*e);
    registry_.remove<ObjectCollider>(*e);

    return Result::ok();
  }

  Result set_cloth_pin_position(uint32_t handle, ConstSpan<float> position) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto pin = registry_.get<Pin>(*e);
    assert(pin);

    if (3 * pin->index.size() != position.size) {
      return Result::error(ErrorCode::IncorrectPinNum);
    }

    pin->position =
        Eigen::Map<const Eigen::VectorXf>(position.data, position.size);

    return Result::ok();
  }

  // Obstacle API
  Result add_obstacle(CollisionConfig collision_config, MeshConfig mesh_config,
                      uint32_t& handle) {
    // make tri mesh
    auto tri_mesh = try_make_obstacle_mesh(mesh_config);
    if (!tri_mesh) {
      return Result::error(ErrorCode::InvalidMesh);
    }

    // make obstacle position
    ObstaclePosition p;
    p.is_static = true;
    p.position = tri_mesh->V.reshaped<Eigen::RowMajor>();
    p.prev_position = p.position;

    auto [h, e] = registry_.add_entity();
    if (h.is_empty()) {
      return Result::error(ErrorCode::TooManyBody);
    }
    assert(e);
    registry_.set<CollisionConfig>(*e, std::move(collision_config));
    registry_.set<TriMesh>(*e, std::move(*tri_mesh));
    registry_.set<ObstaclePosition>(*e, std::move(p));

    handle = h.value;
    return Result::ok();
  }

  Result remove_obstacle(uint32_t handle) {
    auto entity = registry_.get_entity(handle);
    if (!entity) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto obstacle_position = registry_.get<ObstaclePosition>(*entity);
    if (!obstacle_position) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    registry_.remove_entity(handle);
    return Result::ok();
  }

  Result set_obstacle_collision_config(uint32_t handle,
                                       CollisionConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto obstacle_position = registry_.get<ObstaclePosition>(*e);
    if (!obstacle_position) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto collision_config = registry_.get<CollisionConfig>(*e);
    assert(collision_config);

    *collision_config = config;
    return Result::ok();
  }

  Result set_obstacle_position(uint32_t handle, ConstSpan<float> position) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    auto pos = registry_.get<ObstaclePosition>(*e);
    if (!pos) {
      return Result::error(ErrorCode::InvalidHandle);
    }

    if (pos->position.size() != position.size) {
      return Result::error(ErrorCode::IncorrectPositionNum);
    }

    pos->is_static = false;
    std::swap(pos->position, pos->prev_position);
    pos->position =
        Eigen::Map<const Eigen::VectorXf>(position.data, position.size);

    return Result::ok();
  }
};

World::World() : impl_(new WorldImpl) {}
World::~World() = default;
World::World(World&&) = default;
World& World::operator=(World&&) = default;

Result World::set_global_config(GlobalConfig config) {
  return impl_->set_global_config(config);
}
void World::clear() { impl_->clear(); }
Result World::solver_step() { return impl_->solver_step(); }
Result World::solver_reset() { return impl_->solver_reset(); }
Result World::add_cloth(ClothConfig cloth_config,
                        CollisionConfig collision_config,
                        MeshConfig mesh_config, ConstSpan<int> pin_index,
                        uint32_t& handle) {
  return impl_->add_cloth(cloth_config, collision_config, mesh_config,
                          pin_index, handle);
}
Result World::remove_cloth(uint32_t handle) {
  return impl_->remove_cloth(handle);
}
Result World::get_cloth_position(uint32_t handle, Span<float> position) const {
  return impl_->get_cloth_position(handle, position);
}
Result World::set_cloth_config(uint32_t handle, ClothConfig config) {
  return impl_->set_cloth_config(handle, config);
}
Result World::set_cloth_collision_config(uint32_t handle,
                                         CollisionConfig config) {
  return impl_->set_cloth_collision_config(handle, config);
}
Result World::set_cloth_pin_index(uint32_t handle, ConstSpan<int> pin_index) {
  return impl_->set_cloth_pin_index(handle, pin_index);
}
Result World::set_cloth_pin_position(uint32_t handle,
                                     ConstSpan<float> position) {
  return impl_->set_cloth_pin_position(handle, position);
}
Result World::add_obstacle(CollisionConfig collision_config,
                           MeshConfig mesh_config, uint32_t& handle) {
  return impl_->add_obstacle(collision_config, mesh_config, handle);
}
Result World::remove_obstacle(uint32_t handle) {
  return impl_->remove_obstacle(handle);
}
Result World::set_obstacle_collision_config(uint32_t handle,
                                            CollisionConfig config) {
  return impl_->set_obstacle_collision_config(handle, config);
}
Result World::set_obstacle_position(uint32_t handle,
                                    ConstSpan<float> position) {
  return impl_->set_obstacle_position(handle, position);
}

}  // namespace silk
