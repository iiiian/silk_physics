#include "backend/cpu/cpu_backend.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cstring>
#include <span>
#include <utility>

#include "backend/cpu/collision/object_collider.hpp"
#include "backend/cpu/ecs.hpp"
#include "backend/cpu/object_state.hpp"
#include "backend/cpu/obstacle_position.hpp"
#include "backend/cpu/solver/cloth_solver_context.hpp"
#include "backend/cpu/solver/pipeline.hpp"
#include "common/cloth_assembly_l2_cache.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"

namespace silk::cpu {

struct CpuBackend::Impl {
  Registry registry_;
  SolverPipeline solver_pipeline_;
};

CpuBackend::CpuBackend() : impl_(std::make_unique<Impl>()) {}

CpuBackend::~CpuBackend() = default;

Result CpuBackend::set_global_config(GlobalConfig config) {
  auto& c = config;
  impl_->solver_pipeline_.const_acceleration = {
      c.acceleration_x, c.acceleration_y, c.acceleration_z};
  impl_->solver_pipeline_.dt = c.dt;
  impl_->solver_pipeline_.max_outer_iteration = c.max_outer_iteration;
  impl_->solver_pipeline_.max_inner_iteration = c.max_inner_iteration;
  return Result::ok();
}

Result CpuBackend::solver_step() {
  if (!impl_->solver_pipeline_.step(impl_->registry_)) {
    return Result::error(ErrorCode::CholeskyDecompositionFail);
  }
  return Result::ok();
}

Result CpuBackend::solver_reset() {
  impl_->solver_pipeline_.reset(impl_->registry_);
  return Result::ok();
}

Result CpuBackend::add_cloth(ClothConfig cloth_config,
                             CollisionConfig collision_config,
                             MeshConfig mesh_config,
                             std::span<const int> pin_index, uint32_t& handle) {
  auto tri_mesh = make_cloth_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }

  if (pin_index.size() > tri_mesh->V.rows()) {
    return Result::error(ErrorCode::InvalidPin);
  }
  for (int i : pin_index) {
    if (i < 0 || i >= tri_mesh->V.rows()) {
      return Result::error(ErrorCode::InvalidPin);
    }
  }
  Pin p{pin_index, tri_mesh->V};

  uint32_t e = impl_->registry_.make_entity();
  if (e == 0) {
    handle = 0;
    return Result::error(ErrorCode::TooManyBody);
  }
  impl_->registry_.set(e, std::move(cloth_config));
  impl_->registry_.set(e, std::move(collision_config));
  impl_->registry_.set(e, std::move(*tri_mesh));
  impl_->registry_.set(e, std::move(p));

  handle = e;
  return Result::ok();
}

Result CpuBackend::remove_cloth(uint32_t handle) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.nuke_entity(handle);
  return Result::ok();
}

Result CpuBackend::get_cloth_position(uint32_t handle,
                                      std::span<float> position) const {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obj_state = impl_->registry_.get<ObjectState>(handle);
  if (obj_state) {
    if ((int)position.size() < obj_state->state_num) {
      return Result::error(ErrorCode::InvalidPosition);
    }
    memcpy(position.data(), obj_state->curr_state.data(),
           obj_state->state_num * sizeof(float));
    return Result::ok();
  }
  auto mesh = impl_->registry_.get<TriMesh>(handle);
  if (mesh) {
    int state_num = 3 * mesh->V.rows();
    if ((int)position.size() != state_num) {
      return Result::error(ErrorCode::InvalidPosition);
    }
    memcpy(position.data(), mesh->V.data(), state_num * sizeof(float));
    return Result::ok();
  }
  return Result::error(ErrorCode::InvalidHandle);
}

Result CpuBackend::set_cloth_config(uint32_t handle, ClothConfig config) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  *cloth_config = config;
  impl_->registry_.remove<ClothSolverContext>(handle);
  impl_->registry_.remove<ObjectCollider>(handle);
  return Result::ok();
}

Result CpuBackend::set_cloth_collision_config(uint32_t handle,
                                              CollisionConfig config) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto collision_config = impl_->registry_.get<CollisionConfig>(handle);
  assert(collision_config);
  *collision_config = config;
  return Result::ok();
}

Result CpuBackend::set_cloth_pin_index(uint32_t handle,
                                       std::span<const int> pin_index) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto tri_mesh = impl_->registry_.get<TriMesh>(handle);
  assert(tri_mesh);
  for (int i : pin_index) {
    if (i < 0 || i >= tri_mesh->V.rows()) {
      return Result::error(ErrorCode::InvalidPin);
    }
  }

  impl_->registry_.set(handle, Pin{pin_index, tri_mesh->V});
  impl_->registry_.remove<ClothSolverContext>(handle);
  impl_->registry_.remove<ObjectCollider>(handle);
  return Result::ok();
}

Result CpuBackend::set_cloth_pin_position(uint32_t handle,
                                          std::span<const float> position) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto pin = impl_->registry_.get<Pin>(handle);
  assert(pin);
  if (3 * pin->index.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPin);
  }
  pin->prev_position = pin->curr_position;
  if (position.empty()) {
    pin->curr_position.resize(0);
  } else {
    pin->curr_position =
        Eigen::Map<const Eigen::VectorXf>(position.data(), position.size());
  }
  pin->is_static = false;
  pin->is_static_twice = false;
  return Result::ok();
}

Result CpuBackend::add_obstacle(CollisionConfig collision_config,
                                MeshConfig mesh_config, uint32_t& handle) {
  auto tri_mesh = make_obstacle_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }
  ObstaclePosition p;
  p.is_static = false;
  p.is_static_twice = false;
  p.curr_position = tri_mesh->V.reshaped<Eigen::RowMajor>();
  p.prev_position = p.curr_position;
  uint32_t e = impl_->registry_.make_entity();
  if (e == 0) {
    handle = 0;
    return Result::error(ErrorCode::TooManyBody);
  }
  impl_->registry_.set(e, std::move(collision_config));
  impl_->registry_.set(e, std::move(*tri_mesh));
  impl_->registry_.set(e, std::move(p));
  handle = e;
  return Result::ok();
}

Result CpuBackend::remove_obstacle(uint32_t handle) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obstacle_position = impl_->registry_.get<ObstaclePosition>(handle);
  if (!obstacle_position) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.nuke_entity(handle);
  return Result::ok();
}

Result CpuBackend::set_obstacle_collision_config(uint32_t handle,
                                                 CollisionConfig config) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obstacle_position = impl_->registry_.get<ObstaclePosition>(handle);
  if (!obstacle_position) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto collision_config = impl_->registry_.get<CollisionConfig>(handle);
  assert(collision_config);
  *collision_config = config;
  return Result::ok();
}

Result CpuBackend::set_obstacle_position(uint32_t handle,
                                         std::span<const float> position) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto pos = impl_->registry_.get<ObstaclePosition>(handle);
  if (!pos) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  if (pos->curr_position.size() != (int)position.size()) {
    return Result::error(ErrorCode::InvalidPosition);
  }
  pos->is_static = false;
  pos->is_static_twice = false;
  std::swap(pos->curr_position, pos->prev_position);
  pos->curr_position =
      Eigen::Map<const Eigen::VectorXf>(position.data(), position.size());
  return Result::ok();
}

}  // namespace silk::cpu
