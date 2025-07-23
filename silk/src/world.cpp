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

    // nuke entire registry
    registry_.entity.clear();
    registry_.cloth_config.clear();
    registry_.collision_config.clear();
    registry_.tri_mesh.clear();
    registry_.pin_group.clear();
    registry_.solver_data.clear();
    registry_.obstacle.clear();
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
                   MeshConfig mesh_config, ClothHandle& handle) {
    // make tri mesh
    TriMesh m;
    m.V = Eigen::Map<const RMatrix3f>(mesh_config.verts.data,
                                      mesh_config.verts.num, 3);
    m.F = Eigen::Map<const RMatrix3i>(mesh_config.faces.data,
                                      mesh_config.faces.num, 3);
    igl::edges(m.V, m.F, m.E);
    m.avg_edge_length = 0.0f;
    for (int i = 0; i < m.E.rows(); ++i) {
      m.avg_edge_length += (m.V.row(m.E(i, 0)) - m.V.row(m.E(i, 1))).norm();
    }
    m.avg_edge_length /= m.E.rows();

    // make pin group
    PinGroup p;
    p.pin_index = Eigen::Map<const Eigen::VectorXi>(mesh_config.pin_index.data,
                                                    mesh_config.pin_index.num);
    p.pin_value.resize(3 * p.pin_index.size());
    for (int i = 0; i < p.pin_index.size(); ++i) {
      p.pin_value(Eigen::seqN(3 * i, 3)) = m.V.row(i);
    }

    Entity e;
    e.cloth_config = registry_.cloth_config.add(std::move(cloth_config));
    e.collision_config =
        registry_.collision_config.add(std::move(collision_config));
    e.tri_mesh = registry_.tri_mesh.add(std::move(m));
    e.pin_group = registry_.pin_group.add(std::move(p));
    Handle h = registry_.entity.add(e);

    if (h.is_empty() || e.cloth_config.is_empty() ||
        e.collision_config.is_empty() || e.tri_mesh.is_empty() ||
        e.pin_group.is_empty()) {
      ECS_REMOVE(registry_, e, cloth_config);
      ECS_REMOVE(registry_, e, collision_config);
      ECS_REMOVE(registry_, e, tri_mesh);
      ECS_REMOVE(registry_, e, pin_group);
      registry_.entity.remove(h);
      return Result::TooManyBody;
    }

    is_solver_init = false;
    return Result::Success;
  }

  Result remove_cloth(ClothHandle handle) {
    Handle h{handle.value};
    auto cloth = registry_.entity.get(h);
    if (!cloth) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;
    Entity& e = *cloth;
    ECS_REMOVE(registry_, e, cloth_config);
    ECS_REMOVE(registry_, e, collision_config);
    ECS_REMOVE(registry_, e, tri_mesh);
    ECS_REMOVE(registry_, e, pin_group);
    ECS_REMOVE(registry_, e, solver_data);
    ECS_REMOVE(registry_, e, obstacle);
    registry_.entity.remove(h);

    return Result::Success;
  }

  Result get_cloth_position(ClothHandle handle, View<float> position) const {
    auto cloth = registry_.entity.get(Handle{handle.value});
    if (!cloth) {
      return Result::InvalidHandle;
    }

    const Entity& e = *cloth;
    if (is_solver_init) {
      auto solver_data = ECS_GET_PTR(registry_, e, solver_data);
      assert(solver_data);

      if (position.num < solver_data->state_num) {
        return Result::IncorrectPositionNum;
      }

      auto solver_state = solver_.get_solver_state();
      assert((solver_state.size() >=
              solver_data->state_offset + solver_data->state_num));

      memcpy(position.data, solver_state.data() + solver_data->state_offset,
             solver_data->state_num);
      return Result::Success;
    } else {
      auto mesh = ECS_GET_PTR(registry_, e, tri_mesh);
      assert(mesh);

      if (position.num < 3 * mesh->V.rows()) {
        return Result::IncorrectPositionNum;
      }

      memcpy(position.data, mesh->V.data(), 3 * mesh->V.rows());
    }

    return Result::Success;
  }

  Result set_cloth_config(ClothHandle handle, ClothConfig config) {
    auto cloth = registry_.entity.get(Handle{handle.value});
    if (!cloth) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;
    Entity& e = *cloth;
    auto cloth_config = ECS_GET_PTR(registry_, e, cloth_config);
    assert(cloth_config);
    *cloth_config = config;
    return Result::Success;
  }

  Result set_cloth_collision_config(ClothHandle handle,
                                    CollisionConfig config) {
    auto cloth = registry_.entity.get(Handle{handle.value});
    if (!cloth) {
      return Result::InvalidHandle;
    }

    Entity& e = *cloth;
    auto collision_config = ECS_GET_PTR(registry_, e, collision_config);
    assert(collision_config);
    *collision_config = config;
    return Result::Success;
  }

  Result set_cloth_mesh(ClothHandle handle, MeshConfig mesh_config) {
    auto cloth = registry_.entity.get(Handle{handle.value});
    if (!cloth) {
      return Result::InvalidHandle;
    }

    is_solver_init = false;

    // make tri mesh
    Entity& e = *cloth;
    auto tri_mesh = ECS_GET_PTR(registry_, e, tri_mesh);
    assert(tri_mesh);
    tri_mesh->V = Eigen::Map<const RMatrix3f>(mesh_config.verts.data,
                                              mesh_config.verts.num, 3);
    tri_mesh->F = Eigen::Map<const RMatrix3i>(mesh_config.faces.data,
                                              mesh_config.faces.num, 3);
    igl::edges(tri_mesh->V, tri_mesh->F, tri_mesh->E);
    tri_mesh->avg_edge_length = 0.0f;
    for (int i = 0; i < tri_mesh->E.rows(); ++i) {
      tri_mesh->avg_edge_length += (tri_mesh->V.row(tri_mesh->E(i, 0)) -
                                    tri_mesh->V.row(tri_mesh->E(i, 1)))
                                       .norm();
    }
    tri_mesh->avg_edge_length /= tri_mesh->E.rows();

    // make pin group
    auto pin_group = ECS_GET_PTR(registry_, e, pin_group);
    assert(pin_group);
    pin_group->pin_index = Eigen::Map<const Eigen::VectorXi>(
        mesh_config.pin_index.data, mesh_config.pin_index.num);
    pin_group->pin_value.resize(3 * pin_group->pin_index.size());
    for (int i = 0; i < pin_group->pin_index.size(); ++i) {
      pin_group->pin_value(Eigen::seqN(3 * i, 3)) = tri_mesh->V.row(i);
    }
    pin_group->prev_pin_value = {};

    return Result::Success;
  }

  Result set_cloth_pin(ClothHandle handle, ConstView<float> pin) {
    auto cloth = registry_.entity.get(Handle{handle.value});
    if (!cloth) {
      return Result::InvalidHandle;
    }

    Entity& e = *cloth;
    auto pin_group = ECS_GET_PTR(registry_, e, pin_group);
    assert(pin_group);

    if (pin_group->pin_index.size() != pin.num) {
      return Result::IncorrectPinNum;
    }

    pin_group->prev_pin_value = pin_group->prev_pin_value;
    pin_group->pin_value = Eigen::Map<const Eigen::VectorXf>(pin.data, pin.num);

    return Result::Success;
  }

  // Obstacle API
  Result add_obstacle(CollisionConfig collision_config, MeshConfig mesh_config,
                      ObstacleHandle& handle);
  Result remove_obstacle(ObstacleHandle handle);
  Result set_obstacle_collision_config(ObstacleHandle handle,
                                       CollisionConfig config);
  Result set_obstacle_mesh(const ObstacleHandle& handle,
                           MeshConfig mesh_config);
  Result set_obstacle_position(ClothHandle handle, ConstView<float> position);
};

}  // namespace silk
