#include "backend/cuda/cuda_backend.hpp"

#include <cuda_runtime_api.h>

#include <Eigen/Core>
#include <cassert>
#include <cstring>

#include "backend/cuda/collision/object_collider.hpp"
#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/object_state.hpp"
#include "backend/cuda/obstacle_position.hpp"
#include "backend/cuda/solver/pipeline.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"

namespace silk::cuda {

struct CudaBackend::Impl {
  Registry registry_;
  SolverPipeline solver_pipeline_;
};

CudaBackend::CudaBackend() : impl_(std::make_unique<Impl>()) {}

CudaBackend::~CudaBackend() = default;

Result CudaBackend::set_global_config(GlobalConfig config) {
  auto& c = config;
  impl_->solver_pipeline_.const_acceleration = {
      c.acceleration_x, c.acceleration_y, c.acceleration_z};
  impl_->solver_pipeline_.dt = c.dt;
  impl_->solver_pipeline_.max_outer_iteration = c.max_outer_iteration;
  impl_->solver_pipeline_.max_inner_iteration = c.max_inner_iteration;
  return Result::ok();
}

void CudaBackend::clear() {
  impl_->solver_pipeline_.clear(impl_->registry_);
  impl_->registry_.clear();
}

Result CudaBackend::solver_step() {
  if (!impl_->solver_pipeline_.step(impl_->registry_)) {
    return Result::error(ErrorCode::Unknown);
  }
  return Result::ok();
}

Result CudaBackend::solver_reset() {
  impl_->solver_pipeline_.reset(impl_->registry_);
  return Result::ok();
}

Result CudaBackend::add_cloth(ClothConfig cloth_config,
                              CollisionConfig collision_config,
                              MeshConfig mesh_config, ConstSpan<int> pin_index,
                              uint32_t& handle) {
  auto tri_mesh = make_cloth_mesh(mesh_config);
  if (!tri_mesh) {
    handle = 0;
    return Result::error(ErrorCode::InvalidMesh);
  }

  Pin p;
  if (pin_index.data != nullptr && pin_index.size != 0) {
    p.index = Eigen::Map<const Eigen::VectorXi>(pin_index.data, pin_index.size);
    p.position.resize(3 * p.index.size());
    for (int i = 0; i < p.index.size(); ++i) {
      p.position(Eigen::seqN(3 * i, 3)) = tri_mesh->V.row(p.index(i));
    }
  }

  auto [h, e] = impl_->registry_.add_entity();
  if (h.is_empty()) {
    handle = 0;
    return Result::error(ErrorCode::TooManyBody);
  }
  assert(e);
  impl_->registry_.set<ClothConfig>(*e, std::move(cloth_config));
  impl_->registry_.set<CollisionConfig>(*e, std::move(collision_config));
  impl_->registry_.set<TriMesh>(*e, std::move(*tri_mesh));
  impl_->registry_.set<Pin>(*e, std::move(p));

  handle = h.value;
  return Result::ok();
}

Result CudaBackend::remove_cloth(uint32_t handle) {
  auto entity = impl_->registry_.get_entity(Handle(handle));
  if (!entity) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(*entity);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  impl_->registry_.remove_entity(Handle(handle));
  return Result::ok();
}

Result CudaBackend::get_cloth_position(uint32_t handle,
                                       Span<float> position) const {
  const Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obj_state = impl_->registry_.get<ObjectState>(e);
  if (obj_state) {
    if (position.size < obj_state->state_num) {
      return Result::error(ErrorCode::IncorrectPositionNum);
    }
    Eigen::VectorXf curr_state(obj_state->state_num);
    CHECK_CUDA(cudaMemcpy(curr_state.data(), obj_state->d_curr_state,
                          obj_state->state_num * sizeof(float),
                          cudaMemcpyDeviceToHost));

    const TriMesh* mesh = impl_->registry_.get<TriMesh>(e);
    int vert_num =
        (mesh) ? static_cast<int>(mesh->V.rows()) : obj_state->state_num / 3;
    // Map from permuted solver order back to original vertex order.
    if (obj_state->inv_perm.size() == vert_num) {
      for (int v_old = 0; v_old < vert_num; ++v_old) {
        int v_new = obj_state->inv_perm(v_old);
        int dst_offset = 3 * v_old;
        int src_offset = 3 * v_new;
        position.data[dst_offset + 0] = curr_state[src_offset + 0];
        position.data[dst_offset + 1] = curr_state[src_offset + 1];
        position.data[dst_offset + 2] = curr_state[src_offset + 2];
      }
    } else {
      // Fallback: treat state as already in user order.
      memcpy(position.data, curr_state.data(),
             obj_state->state_num * sizeof(float));
    }
    return Result::ok();
  }
  auto mesh = impl_->registry_.get<TriMesh>(e);
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

Result CudaBackend::set_cloth_config(uint32_t handle, ClothConfig config) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(*e);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  *cloth_config = config;
  impl_->registry_.remove<ClothSolverContext>(*e);
  impl_->registry_.remove<ObjectCollider>(*e);
  return Result::ok();
}

Result CudaBackend::set_cloth_collision_config(uint32_t handle,
                                               CollisionConfig config) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(*e);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto collision_config = impl_->registry_.get<CollisionConfig>(*e);
  assert(collision_config);
  *collision_config = config;
  return Result::ok();
}

Result CudaBackend::set_cloth_pin_index(uint32_t handle,
                                        ConstSpan<int> pin_index) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(*e);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto tri_mesh = impl_->registry_.get<TriMesh>(*e);
  assert(tri_mesh);
  auto pin = impl_->registry_.get<Pin>(*e);
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
  impl_->registry_.remove<ClothSolverContext>(*e);
  impl_->registry_.remove<ObjectCollider>(*e);
  return Result::ok();
}

Result CudaBackend::set_cloth_pin_position(uint32_t handle,
                                           ConstSpan<float> position) {
  Entity* e = impl_->registry_.get_entity(Handle(handle));
  if (!e) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto cloth_config = impl_->registry_.get<ClothConfig>(*e);
  if (!cloth_config) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto pin = impl_->registry_.get<Pin>(*e);
  assert(pin);
  if (3 * pin->index.size() != position.size) {
    return Result::error(ErrorCode::IncorrectPinNum);
  }
  pin->position =
      Eigen::Map<const Eigen::VectorXf>(position.data, position.size);
  return Result::ok();
}

Result CudaBackend::add_obstacle(CollisionConfig collision_config,
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
  auto [h, e] = impl_->registry_.add_entity();
  if (h.is_empty()) {
    handle = 0;
    return Result::error(ErrorCode::TooManyBody);
  }
  assert(e);
  impl_->registry_.set<CollisionConfig>(*e, std::move(collision_config));
  impl_->registry_.set<TriMesh>(*e, std::move(*tri_mesh));
  impl_->registry_.set<ObstaclePosition>(*e, std::move(p));
  handle = h.value;
  return Result::ok();
}

Result CudaBackend::remove_obstacle(uint32_t handle) {
  auto entity = impl_->registry_.get_entity(Handle(handle));
  if (!entity) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  auto obstacle_position = impl_->registry_.get<ObstaclePosition>(*entity);
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
  auto obstacle_position = impl_->registry_.get<ObstaclePosition>(*e);
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
  auto pos = impl_->registry_.get<ObstaclePosition>(*e);
  if (!pos) {
    return Result::error(ErrorCode::InvalidHandle);
  }
  if (pos->curr_position.size() != position.size) {
    return Result::error(ErrorCode::IncorrectPositionNum);
  }
  pos->is_static = false;
  pos->is_static_twice = false;
  std::swap(pos->curr_position, pos->prev_position);
  pos->curr_position =
      Eigen::Map<const Eigen::VectorXf>(position.data, position.size);
  return Result::ok();
}

}  // namespace silk::cuda
