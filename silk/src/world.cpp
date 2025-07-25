#include <igl/edges.h>

#include <Eigen/Core>
#include <cassert>
#include <cstring>

#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "solver.hpp"

namespace silk {

std::string to_string(Result result) {
  switch (result) {
    case Result::Success: {
      return "Success";
    }
    case Result::InvalidConfig: {
      return "InvalidConfig";
    }
    case Result::TooManyBody: {
      return "TooManyBody";
    }
    case Result::InvalidHandle: {
      return "InvalidHandle";
    }
    default:
      assert(false && "unknown result");
      return "Unknown";
  }
}

class World::WorldImpl {
 private:
  bool is_solver_init = false;
  Registry registry_;
  Solver solver_;
  CollisionPipeline collision_pipeline_;

 public:
  // Global API
  Result set_global_config(GlobalConfig config) {
    is_solver_init = false;

    auto& c = config;
    solver_.const_acceleration = {c.acceleration_x, c.acceleration_y,
                                  c.acceleration_z};
    solver_.max_iteration = c.max_iteration;
    solver_.r = c.r;
    solver_.dt = c.dt;
    solver_.ccd_walkback = c.ccd_walkback;
    collision_pipeline_.toi_tolerance = c.toi_tolerance;
    collision_pipeline_.toi_refine_it = c.toi_refine_iteration;
    collision_pipeline_.eps = c.eps;

    return Result::Success;
  }

  void clear() {
    is_solver_init = false;
    solver_.clear();
    registry_.clear();
  }

  // Solver API
  Result solver_init() {
    if (!solver_.init(registry_)) {
      return Result::EigenDecompositionFail;
    }
    is_solver_init = true;
    return Result::Success;
  }

  Result solver_step() {
    if (!is_solver_init) {
      return Result::NeedInitSolverFirst;
    }

    solver_.step(registry_, collision_pipeline_);

    for (Entity& e : registry_.get_all_entities()) {
      auto obstacle_position = registry_.get<ObstaclePosition>(e);
      if (obstacle_position) {
        obstacle_position->prev_position = {};
        obstacle_position->is_static = true;
      }
    }

    return Result::Success;
  }

  Result solver_reset() {
    if (!is_solver_init) {
      return Result::NeedInitSolverFirst;
    }
    solver_.reset();
    return Result::Success;
  }

  // cloth API
  Result add_cloth(ClothConfig cloth_config, CollisionConfig collision_config,
                   MeshConfig mesh_config, ConstSpan<int> pin_index,
                   uint32_t& handle) {
    // make tri mesh
    TriMesh m;
    m.V = Eigen::Map<const RMatrix3f>(mesh_config.verts.data,
                                      mesh_config.verts.num, 3);
    m.F = Eigen::Map<const RMatrix3i>(mesh_config.faces.data,
                                      mesh_config.faces.num, 3);
    igl::edges(m.F, m.E);
    m.avg_edge_length = 0.0f;
    for (int i = 0; i < m.E.rows(); ++i) {
      m.avg_edge_length += (m.V.row(m.E(i, 0)) - m.V.row(m.E(i, 1))).norm();
    }
    m.avg_edge_length /= m.E.rows();

    // make pin
    Pin p;
    if (pin_index.data != nullptr && pin_index.num != 0) {
      p.index =
          Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.num);
      p.position.resize(3 * p.index.size());
      for (int i = 0; i < p.index.size(); ++i) {
        p.position(Eigen::seqN(3 * i, 3)) = m.V.row(i);
      }
    }

    auto [h, e] = registry_.add_entity();
    if (h.is_empty()) {
      return Result::TooManyBody;
    }
    assert(e);
    registry_.set<ClothConfig>(*e, std::move(cloth_config));
    registry_.set<CollisionConfig>(*e, std::move(collision_config));
    registry_.set<TriMesh>(*e, std::move(m));
    registry_.set<Pin>(*e, std::move(p));

    handle = h.value;
    is_solver_init = false;
    return Result::Success;
  }

  Result remove_cloth(uint32_t handle) {
    auto entity = registry_.get_entity(handle);
    if (!entity) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*entity);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;
    registry_.remove_entity(handle);

    return Result::Success;
  }

  Result get_cloth_position(uint32_t handle, Span<float> position) const {
    const Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    if (is_solver_init) {
      auto solver_data = registry_.get<SolverData>(*e);
      assert(solver_data);

      if (position.num < solver_data->state_num) {
        return Result::IncorrectPositionNum;
      }

      auto solver_state = solver_.get_solver_state();
      assert((solver_state.size() >=
              solver_data->state_offset + solver_data->state_num));

      memcpy(position.data, solver_state.data() + solver_data->state_offset,
             solver_data->state_num * sizeof(float));
      return Result::Success;
    } else {
      auto mesh = registry_.get<TriMesh>(*e);
      assert(mesh);

      if (position.num < 3 * mesh->V.rows()) {
        return Result::IncorrectPositionNum;
      }

      memcpy(position.data, mesh->V.data(), 3 * mesh->V.rows() * sizeof(float));
    }

    return Result::Success;
  }

  Result set_cloth_config(uint32_t handle, ClothConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;
    *cloth_config = config;

    // remove outdated components
    registry_.remove<SolverData>(*e);
    registry_.remove<ObjectCollider>(*e);

    return Result::Success;
  }

  Result set_cloth_collision_config(uint32_t handle, CollisionConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    auto collision_config = registry_.get<CollisionConfig>(*e);
    assert(collision_config);

    *collision_config = config;
    return Result::Success;
  }

  Result set_cloth_mesh_config(uint32_t handle, MeshConfig mesh_config,
                               ConstSpan<int> pin_index) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    auto m = registry_.get<TriMesh>(*e);
    auto p = registry_.get<Pin>(*e);
    assert(m);
    assert(p);

    // make tri mesh
    m->V = Eigen::Map<const RMatrix3f>(mesh_config.verts.data,
                                       mesh_config.verts.num, 3);
    m->F = Eigen::Map<const RMatrix3i>(mesh_config.faces.data,
                                       mesh_config.faces.num, 3);
    igl::edges(m->F, m->E);
    m->avg_edge_length = 0.0f;
    for (int i = 0; i < m->E.rows(); ++i) {
      m->avg_edge_length +=
          (m->V.row(m->E(i, 0)) - m->V.row(m->E(i, 1))).norm();
    }
    m->avg_edge_length /= m->E.rows();

    // make pin
    if (pin_index.data != nullptr && pin_index.num != 0) {
      p->index =
          Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.num);
      p->position.resize(3 * p->index.size());
      for (int i = 0; i < p->index.size(); ++i) {
        p->position(Eigen::seqN(3 * i, 3)) = m->V.row(i);
      }
    } else {
      p = {};
    }

    is_solver_init = false;
    return Result::Success;
  }

  Result set_cloth_pin_index(uint32_t handle, ConstSpan<int> pin_index) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;

    // make tri mesh
    auto tri_mesh = registry_.get<TriMesh>(*e);
    assert(tri_mesh);

    // make pin
    auto pin = registry_.get<Pin>(*e);
    assert(pin);
    if (pin_index.data != nullptr && pin_index.num != 0) {
      pin->index =
          Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.num);
      pin->position.resize(3 * pin->index.size());
      for (int i = 0; i < pin->index.size(); ++i) {
        pin->position(Eigen::seqN(3 * i, 3)) = tri_mesh->V.row(i);
      }
    } else {
      pin = {};
    }

    // remove outdated components
    registry_.remove<SolverData>(*e);
    registry_.remove<ObjectCollider>(*e);

    return Result::Success;
  }

  Result set_cloth_pin_position(uint32_t handle, ConstSpan<float> position) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto cloth_config = registry_.get<ClothConfig>(*e);
    if (!cloth_config) {
      return Result::InvalidHandle;
    }

    auto pin = registry_.get<Pin>(*e);
    assert(pin);

    if (3 * pin->index.size() != position.num) {
      return Result::IncorrectPinNum;
    }

    pin->position =
        Eigen::Map<const Eigen::VectorXf>(position.data, position.num);

    // remove outdated components
    registry_.remove<SolverData>(*e);
    registry_.remove<ObjectCollider>(*e);

    return Result::Success;
  }

  // Obstacle API
  Result add_obstacle(CollisionConfig collision_config, MeshConfig mesh_config,
                      uint32_t& handle) {
    // make tri mesh
    TriMesh m;
    m.V = Eigen::Map<const RMatrix3f>(mesh_config.verts.data,
                                      mesh_config.verts.num, 3);
    m.F = Eigen::Map<const RMatrix3i>(mesh_config.faces.data,
                                      mesh_config.faces.num, 3);
    igl::edges(m.F, m.E);
    m.avg_edge_length = 0.0f;
    for (int i = 0; i < m.E.rows(); ++i) {
      m.avg_edge_length += (m.V.row(m.E(i, 0)) - m.V.row(m.E(i, 1))).norm();
    }
    m.avg_edge_length /= m.E.rows();

    // make obstacle position
    ObstaclePosition p;
    p.is_static = true;
    p.position = m.V.reshaped<Eigen::RowMajor>();

    auto [h, e] = registry_.add_entity();
    if (h.is_empty()) {
      return Result::TooManyBody;
    }
    assert(e);
    registry_.set<CollisionConfig>(*e, std::move(collision_config));
    registry_.set<TriMesh>(*e, std::move(m));
    registry_.set<ObstaclePosition>(*e, std::move(p));

    handle = h.value;
    return Result::Success;
  }

  Result remove_obstacle(uint32_t handle) {
    auto entity = registry_.get_entity(handle);
    if (!entity) {
      return Result::InvalidHandle;
    }

    auto obstacle_position = registry_.get<ObstaclePosition>(*entity);
    if (!obstacle_position) {
      return Result::InvalidHandle;
    }

    registry_.remove_entity(handle);
    return Result::Success;
  }

  Result set_obstacle_collision_config(uint32_t handle,
                                       CollisionConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto obstacle_position = registry_.get<ObstaclePosition>(*e);
    if (!obstacle_position) {
      return Result::InvalidHandle;
    }

    auto collision_config = registry_.get<CollisionConfig>(*e);
    assert(collision_config);

    *collision_config = config;
    return Result::Success;
  }

  Result set_obstacle_mesh_config(uint32_t handle, MeshConfig config) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto obstacle_position = registry_.get<ObstaclePosition>(*e);
    if (!obstacle_position) {
      return Result::InvalidHandle;
    }

    auto m = registry_.get<TriMesh>(*e);
    assert(m);

    m->V = Eigen::Map<const RMatrix3f>(config.verts.data, config.verts.num, 3);
    m->F = Eigen::Map<const RMatrix3i>(config.faces.data, config.faces.num, 3);
    igl::edges(m->F, m->E);
    m->avg_edge_length = 0.0f;
    for (int i = 0; i < m->E.rows(); ++i) {
      m->avg_edge_length +=
          (m->V.row(m->E(i, 0)) - m->V.row(m->E(i, 1))).norm();
    }
    m->avg_edge_length /= m->E.rows();

    // make obstacle position
    ObstaclePosition p;
    p.is_static = true;
    p.position = m->V.reshaped<Eigen::RowMajor>();

    return Result::Success;
  }

  Result set_obstacle_position(uint32_t handle, ConstSpan<float> position) {
    Entity* e = registry_.get_entity(handle);
    if (!e) {
      return Result::InvalidHandle;
    }

    auto pos = registry_.get<ObstaclePosition>(*e);
    if (!pos) {
      return Result::InvalidHandle;
    }

    if (pos->position.size() != position.num) {
      return Result::IncorrectPinNum;
    }

    pos->is_static = false;
    std::swap(pos->position, pos->prev_position);
    pos->position =
        Eigen::Map<const Eigen::VectorXf>(position.data, position.num);

    return Result::Success;
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
Result World::solver_init() { return impl_->solver_init(); }
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

Result World::set_cloth_mesh_config(uint32_t handle, MeshConfig mesh_config,
                                    ConstSpan<int> pin_index) {
  return impl_->set_cloth_mesh_config(handle, mesh_config, pin_index);
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
Result World::set_obstacle_mesh_config(uint32_t handle, MeshConfig config) {
  return impl_->set_obstacle_mesh_config(handle, config);
}
Result World::set_obstacle_position(uint32_t handle,
                                    ConstSpan<float> position) {
  return impl_->set_obstacle_position(handle, position);
}

}  // namespace silk
