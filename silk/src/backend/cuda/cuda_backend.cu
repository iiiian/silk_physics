#include "backend/cuda/cuda_backend.hpp"

#include <cuda_runtime_api.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cuda/algorithm>
#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/stream>
#include <stdexcept>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/main_loop.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/pin.hpp"
#include "backend/cuda/solver/cloth_admm_helper.cuh"
#include "common/cloth_assembly_l2_cache.hpp"
#include "common/config_plus.hpp"
#include "common/initial_state.hpp"
#include "common/mesh.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

class CudaBackend::Impl {
 public:
  // Cuda resources MUST BE the first in member declaration.
  // Else they will be destroyed before ref.
  std::optional<cu::device_ref> device;
  std::optional<cu::stream> stream;
  mutable std::optional<cu::device_memory_pool> mr;

  ObjRegistry registry_;
  MainLoop main_loop_;

  Impl() {
    if (cu::devices.size() == 0) {
      throw std::runtime_error("No Nvidia GPU!!");
    }

    device.emplace(cu::devices[0]);
    stream.emplace(*device);
    mr.emplace(*device);
  }

  CudaRuntime get_runtime() const {
    return CudaRuntime{.stream = *stream, .mr = mr->as_ref()};
  }

  bool is_obstacle(uint32_t entity) {
    return !registry_.has_component<ClothConfig>(entity) &&
           registry_.has_entity(entity);
  }
};

CudaBackend::CudaBackend() : impl_(std::make_unique<Impl>()) {}

CudaBackend::~CudaBackend() = default;

Result CudaBackend::set_global_config(GlobalConfig config) {
  auto& c = config;
  impl_->main_loop_.const_acceleration = {c.acceleration_x, c.acceleration_y,
                                          c.acceleration_z};
  impl_->main_loop_.dt = c.dt;
  impl_->main_loop_.max_outer_iteration = c.max_outer_iteration;
  impl_->main_loop_.max_inner_iteration = c.max_inner_iteration;
  return Result::ok();
}

Result CudaBackend::solver_step() {
  auto err = impl_->main_loop_.step(impl_->registry_, impl_->get_runtime());
  if (err) {
    // TODO: better err message.
    return Result::error(ErrorCode::Unknown);
  }
  return Result::ok();
}

Result CudaBackend::solver_reset() {
  impl_->registry_.remove_all_components<ClothADMMHelper>();
  impl_->registry_.remove_all_components<ClothAssemblyL1Cache>();
  impl_->registry_.remove_all_components<ClothAssemblyL2Cache>();
  impl_->registry_.remove_all_components<PhysicalState>();
  impl_->registry_.remove_all_components<ObjectCollider>();
  impl_->registry_.remove_all_components<PinPosition>();

  return Result::ok();
}

// TODO
Result CudaBackend::add_cloth(ClothConfig cloth_config,
                              CollisionConfig collision_config,
                              MeshConfig mesh_config,
                              std::span<const int> pin_index,
                              uint32_t& handle) {
  auto tri_mesh = make_cloth_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }

  InitialState init_state{tri_mesh->V};

  if (pin_index.size() > tri_mesh->V.rows()) {
    return Result::error(ErrorCode::InvalidPin);
  }
  for (int i : pin_index) {
    if (i < 0 || i >= tri_mesh->V.rows()) {
      return Result::error(ErrorCode::InvalidPin);
    }
  }
  PinIndex pin{pin_index};
  CollisionConfigPlus collision_config_plus{
      .is_updated = true,
      .is_collision_on = collision_config.is_collision_on,
      .is_self_collision_on = collision_config.is_self_collision_on,
      .group = collision_config.group,
      .restitution = collision_config.restitution,
      .friction = collision_config.friction,
  };

  uint32_t e = impl_->registry_.make_entity();
  impl_->registry_.set(e, std::move(cloth_config));
  impl_->registry_.set(e, std::move(collision_config_plus));
  impl_->registry_.set(e, std::move(*tri_mesh));
  impl_->registry_.set(e, std::move(init_state));
  impl_->registry_.set(e, std::move(pin));

  handle = e;
  return Result::ok();
}

Result CudaBackend::remove_cloth(uint32_t handle) {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.nuke_entity(handle);
  return Result::ok();
}

Result CudaBackend::get_cloth_position(uint32_t handle,
                                       std::span<float> position) const {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  // Get position from physical state.
  auto state = impl_->registry_.get<PhysicalState>(handle);
  if (state) {
    if (position.size() != state->state_num) {
      return Result::error(ErrorCode::InvalidPosition);
    }

    CudaRuntime rt = impl_->get_runtime();
    auto part = impl_->registry_.get<MeshPartition>(handle);
    assert(part);

    auto tmp = alloc<float>(rt, position.size());
    part->inv_permute(*state->curr_state, tmp, rt);
    cu::copy_bytes(rt.stream, tmp, position);
    rt.stream.sync();

    return Result::ok();
  }

  // Get position from init state.
  auto init_pos = impl_->registry_.get<InitialState>(handle);
  assert(init_pos);
  if (init_pos->position.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPosition);
  }
  std::ranges::copy(init_pos->position, position.begin());
  return Result::ok();
}

Result CudaBackend::set_cloth_config(uint32_t handle, ClothConfig config) {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.remove_deps_then_set(handle, config);
  return Result::ok();
}

Result CudaBackend::set_cloth_collision_config(uint32_t handle,
                                               CollisionConfig config) {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  CollisionConfigPlus config_plus{
      .is_updated = true,
      .is_collision_on = config.is_collision_on,
      .is_self_collision_on = config.is_self_collision_on,
      .group = config.group,
      .restitution = config.restitution,
      .friction = config.friction,
  };
  impl_->registry_.remove_deps_then_set(handle, std::move(config_plus));
  return Result::ok();
}

Result CudaBackend::set_cloth_pin_index(uint32_t handle,
                                        std::span<const int> pin_index) {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  auto tri_mesh = impl_->registry_.get<TriMesh>(handle);
  assert(tri_mesh);
  for (int i : pin_index) {
    if (i < 0 || i >= tri_mesh->V.rows()) {
      return Result::error(ErrorCode::InvalidPin);
    }
  }

  impl_->registry_.remove_deps_then_set(handle, PinIndex{pin_index});
  return Result::ok();
}

Result CudaBackend::set_cloth_pin_position(uint32_t handle,
                                           std::span<const float> position) {
  if (!impl_->registry_.has_component<ClothConfig>(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  auto pin_index = impl_->registry_.get<PinIndex>(handle);
  assert(pin_index);
  if (3 * pin_index->index.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPin);
  }

  auto pin_pos = impl_->registry_.get<PinPosition>(handle);
  if (pin_pos) {
    pin_pos->prev_position = pin_pos->curr_position;
    pin_pos->curr_position.assign(position.begin(), position.end());
    pin_pos->is_static = false;
    pin_pos->is_static_twice = false;
  } else {
    impl_->registry_.remove_deps_then_set(handle,
                                          PinPosition::from_position(position));
  }
  return Result::ok();
}

Result CudaBackend::add_obstacle(CollisionConfig collision_config,
                                 MeshConfig mesh_config, uint32_t& handle) {
  auto tri_mesh = make_obstacle_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }

  InitialState init_state{tri_mesh->V};

  PinIndex pin_index;
  pin_index.is_all_pinned = true;
  CollisionConfigPlus collision_config_plus{
      .is_updated = true,
      .is_collision_on = collision_config.is_collision_on,
      .is_self_collision_on = collision_config.is_self_collision_on,
      .group = collision_config.group,
      .restitution = collision_config.restitution,
      .friction = collision_config.friction,
  };

  uint32_t e = impl_->registry_.make_entity();
  impl_->registry_.set(e, std::move(collision_config_plus));
  impl_->registry_.set(e, std::move(*tri_mesh));
  impl_->registry_.set(e, std::move(init_state));
  impl_->registry_.set(e, std::move(pin_index));

  handle = e;
  return Result::ok();
}

Result CudaBackend::remove_obstacle(uint32_t handle) {
  if (!impl_->is_obstacle(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.nuke_entity(handle);
  return Result::ok();
}

Result CudaBackend::set_obstacle_collision_config(uint32_t handle,
                                                  CollisionConfig config) {
  if (!impl_->is_obstacle(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  CollisionConfigPlus config_plus{
      .is_updated = true,
      .is_collision_on = config.is_collision_on,
      .is_self_collision_on = config.is_self_collision_on,
      .group = config.group,
      .restitution = config.restitution,
      .friction = config.friction,
  };
  impl_->registry_.remove_deps_then_set(handle, std::move(config_plus));
  return Result::ok();
}

Result CudaBackend::set_obstacle_position(uint32_t handle,
                                          std::span<const float> position) {
  if (!impl_->is_obstacle(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  auto pos = impl_->registry_.get<PinPosition>(handle);
  assert(pos);
  if (pos->curr_position.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPosition);
  }

  pos->is_static = false;
  pos->is_static_twice = false;
  std::swap(pos->curr_position, pos->prev_position);
  pos->curr_position.assign(position.begin(), position.end());
  return Result::ok();
}

}  // namespace silk::cuda
