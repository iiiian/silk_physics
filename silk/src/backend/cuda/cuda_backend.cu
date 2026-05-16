#include "backend/cuda/cuda_backend.hpp"

#include <cuda_runtime_api.h>

#include <Eigen/Core>
#include <cassert>
#include <cuda/algorithm>
#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/stream>
#include <std/algorithm>
#include <stdexcept>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/main_loop.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "backend/cuda/pin.hpp"
#include "backend/cuda/physical_state.cuh"
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
    auto ref = mr->as_ref();  // resource_ref can't bind to rvalue.
    return CudaRuntime{.stream = *stream, .mr = ref};
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
  impl_->registry_.remove_all_components<ClothAssemblyL2Cache>();
  impl_->registry_.remove_all_components<PhysicalState>();
  impl_->registry_.remove_all_components<collision::ObjectCollider>();
  impl_->registry_.remove_all_components<assembly::ClothAssemblyL1Cache>();
  impl_->registry_.remove_all_components<solver::ClothADMMHelper>();

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
  PinPosition pin_position{pin, tri_mesh->V};

  uint32_t e = impl_->registry_.make_entity();
  impl_->registry_.set(e, std::move(cloth_config));
  impl_->registry_.set(e, std::move(collision_config));
  impl_->registry_.set(e, std::move(*tri_mesh));
  impl_->registry_.set(e, std::move(init_state));
  impl_->registry_.set(e, std::move(pin));
  impl_->registry_.set(e, std::move(pin_position));

  handle = e;
  return Result::ok();
}

Result CudaBackend::remove_cloth(uint32_t handle) {
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

Result CudaBackend::get_cloth_position(uint32_t handle,
                                       std::span<float> position) const {
  if (!impl_->registry_.has_entity(handle)) {
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

    return Result::ok();
  }

  // Get position from init state.
  auto init_pos = impl_->registry_.get<InitialState>(handle);
  assert(init_pos);
  if (init_pos->position.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPosition);
  }
  std::ranges::copy(init_pos.position, position);
  return Result::ok();
}

Result CudaBackend::set_cloth_config(uint32_t handle, ClothConfig config) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }

  *cloth_config = config;
  impl_->registry_.remove<collision::ObjectCollider>(handle);
  impl_->registry_.remove<ClothAssemblyL2Cache>(handle);
  impl_->registry_.remove<assembly::ClothAssemblyL1Cache>(handle);
  impl_->registry_.remove<solver::ClothADMMHelper>(handle);
  return Result::ok();
}

Result CudaBackend::set_cloth_collision_config(uint32_t handle,
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

Result CudaBackend::set_cloth_pin_index(uint32_t handle,
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

  PinIndex pin{pin_index};
  PinPosition pin_position{pin, tri_mesh->V};
  impl_->registry_.set(handle, std::move(pin));
  impl_->registry_.set(handle, std::move(pin_position));
  impl_->registry_.remove<assembly::ClothAssemblyL1Cache>(handle);
  impl_->registry_.remove<collision::ObjectCollider>(handle);
  return Result::ok();
}

Result CudaBackend::set_cloth_pin_position(uint32_t handle,
                                           std::span<const float> position) {
  if (!impl_->registry_.has_entity(handle)) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(handle);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto pin_index = impl_->registry_.get<PinIndex>(handle);
  auto pin_position = impl_->registry_.get<PinPosition>(handle);
  assert(pin_index && pin_position);
  if (3 * pin_index->index.size() != position.size()) {
    return Result::error(ErrorCode::InvalidPin);
  }
  pin_position->prev_position = pin_position->curr_position;
  pin_position->curr_position.assign(position.begin(), position.end());
  pin_position->is_static = false;
  pin_position->is_static_twice = false;
  return Result::ok();
}

Result CudaBackend::add_obstacle(CollisionConfig collision_config,
                                 MeshConfig mesh_config, uint32_t& handle) {
  auto tri_mesh = make_obstacle_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }
  PinPosition p{tri_mesh->V};
  p.is_static = false;
  p.is_static_twice = false;
  auto [h, e] = impl_->registry_.add_entity();
  if (h.is_empty()) {
    handle = 0;
    return Result::error(ErrorCode::TooManyBody);
  }
  assert(e);
  impl_->registry_.set<CollisionConfig>(*e, std::move(collision_config));
  impl_->registry_.set<TriMesh>(*e, std::move(*tri_mesh));
  impl_->registry_.set<PinPosition>(*e, std::move(p));
  handle = h.value;
  return Result::ok();
}

Result CudaBackend::remove_obstacle(uint32_t handle) {
  auto entity = impl_->registry_.get_entity(Handle(handle));
  if (!entity) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obstacle_position = impl_->registry_.get<PinPosition>(*entity);
  if (!obstacle_position) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.remove_entity(Handle(handle));
  return Result::ok();
}

Result CudaBackend::set_obstacle_collision_config(uint32_t handle,
                                                  CollisionConfig config) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obstacle_position = impl_->registry_.get<PinPosition>(*e);
  if (!obstacle_position) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto collision_config = impl_->registry_.get<CollisionConfig>(*e);
  assert(collision_config);
  *collision_config = config;
  return Result::ok();
}

Result CudaBackend::set_obstacle_position(uint32_t handle,
                                          ConstSpan<float> position) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto pos = impl_->registry_.get<PinPosition>(*e);
  if (!pos) {
    return Result::error(ErrorCode::InvalidHandle);
  }
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
