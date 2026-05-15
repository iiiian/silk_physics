#include "backend/cpu/cpu_backend.hpp"
#include "common/backend.hpp"
#include "common/check_cuda_support.hpp"
#ifdef SILK_WITH_CUDA
#include "backend/cuda/cuda_backend.hpp"
#endif
#include <memory>

#include "common/backend.hpp"
#include "silk/silk.hpp"

namespace silk {

class World::Impl {
 public:
  std::unique_ptr<IBackend> backend;
};

World::World() : impl_(std::make_unique<Impl>()) {}

World::~World() = default;

World::World(World&&) = default;

World& World::operator=(World&&) = default;

Result World::init(Backend backend) {
  switch (backend) {
    case Backend::CPU: {
      impl_->backend = std::make_unique<cpu::CpuBackend>();
      return Result::ok();
    }
    case Backend::GPU: {
      if (!check_cuda_support()) {
        return Result::error(ErrorCode::NoCudaSupport,
                             "CUDA runtime/device not available");
      }
#ifdef SILK_WITH_CUDA
      impl_->backend = std::make_unique<cuda::CudaBackend>();
      return Result::ok();
#endif
    }
  }
  return Result::error(ErrorCode::InvalidConfig, "Unknown backend");
}

Result World::set_global_config(GlobalConfig config) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_global_config(config);
}

void World::clear() { impl_->backend = {}; }

Result World::solver_step() {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->solver_step();
}

Result World::solver_reset() {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->solver_reset();
}

Result World::add_cloth(ClothConfig cloth_config,
                        CollisionConfig collision_config,
                        MeshConfig mesh_config, ConstSpan<int> pin_index,
                        uint32_t& handle) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->add_cloth(cloth_config, collision_config, mesh_config,
                                   pin_index, handle);
}

Result World::remove_cloth(uint32_t handle) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->remove_cloth(handle);
}

Result World::get_cloth_position(uint32_t handle, Span<float> position) const {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->get_cloth_position(handle, position);
}

Result World::set_cloth_config(uint32_t handle, ClothConfig config) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_cloth_config(handle, config);
}

Result World::set_cloth_collision_config(uint32_t handle,
                                         CollisionConfig config) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_cloth_collision_config(handle, config);
}

Result World::set_cloth_pin_index(uint32_t handle, ConstSpan<int> pin_index) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_cloth_pin_index(handle, pin_index);
}

Result World::set_cloth_pin_position(uint32_t handle,
                                     ConstSpan<float> position) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_cloth_pin_position(handle, position);
}

Result World::add_obstacle(CollisionConfig collision_config,
                           MeshConfig mesh_config, uint32_t& handle) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->add_obstacle(collision_config, mesh_config, handle);
}

Result World::remove_obstacle(uint32_t handle) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->remove_obstacle(handle);
}

Result World::set_obstacle_collision_config(uint32_t handle,
                                            CollisionConfig config) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_obstacle_collision_config(handle, config);
}

Result World::set_obstacle_position(uint32_t handle,
                                    ConstSpan<float> position) {
  if (!impl_->backend) {
    return Result::error(ErrorCode::NotInitialized);
  }
  return impl_->backend->set_obstacle_position(handle, position);
}

}  // namespace silk
