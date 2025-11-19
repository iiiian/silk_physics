// World facade delegating to selected backend.

#include "backend/cpu/cpu_backend.hpp"
#include "common/backend.hpp"
#include "common/check_cuda_support.hpp"
#ifdef SILK_WITH_CUDA
#include "backend/cuda/cuda_backend.hpp"
#endif
#include "silk/silk.hpp"

namespace silk {

World::World() : backend_(nullptr) {
  // Default to CPU backend
  (void)set_backend(Backend::Cpu);
}

World::~World() = default;

World::World(World&&) = default;

World& World::operator=(World&&) = default;

Result World::set_backend(Backend backend_kind) {
  switch (backend_kind) {
    case Backend::Cpu: {
      backend_ = std::make_unique<cpu::CpuBackend>();
      return Result::ok();
    }
    case Backend::Gpu: {
      if (!check_cuda_support()) {
        return Result::error(ErrorCode::NoCudaSupport,
                             "CUDA runtime/device not available");
      }
#ifdef SILK_WITH_CUDA
      backend_ = std::make_unique<cuda::CudaBackend>();
      return Result::ok();
#endif
    }
  }
  return Result::error(ErrorCode::InvalidConfig, "Unknown backend");
}

Result World::set_global_config(GlobalConfig config) {
  return backend_->set_global_config(config);
}
void World::clear() { backend_->clear(); }

Result World::solver_step() { return backend_->solver_step(); }

Result World::solver_reset() { return backend_->solver_reset(); }

Result World::add_cloth(ClothConfig cloth_config,
                        CollisionConfig collision_config,
                        MeshConfig mesh_config, ConstSpan<int> pin_index,
                        uint32_t& handle) {
  return backend_->add_cloth(cloth_config, collision_config, mesh_config,
                             pin_index, handle);
}

Result World::remove_cloth(uint32_t handle) {
  return backend_->remove_cloth(handle);
}

Result World::get_cloth_position(uint32_t handle, Span<float> position) const {
  return backend_->get_cloth_position(handle, position);
}

Result World::set_cloth_config(uint32_t handle, ClothConfig config) {
  return backend_->set_cloth_config(handle, config);
}

Result World::set_cloth_collision_config(uint32_t handle,
                                         CollisionConfig config) {
  return backend_->set_cloth_collision_config(handle, config);
}

Result World::set_cloth_pin_index(uint32_t handle, ConstSpan<int> pin_index) {
  return backend_->set_cloth_pin_index(handle, pin_index);
}

Result World::set_cloth_pin_position(uint32_t handle,
                                     ConstSpan<float> position) {
  return backend_->set_cloth_pin_position(handle, position);
}

Result World::add_obstacle(CollisionConfig collision_config,
                           MeshConfig mesh_config, uint32_t& handle) {
  return backend_->add_obstacle(collision_config, mesh_config, handle);
}

Result World::remove_obstacle(uint32_t handle) {
  return backend_->remove_obstacle(handle);
}

Result World::set_obstacle_collision_config(uint32_t handle,
                                            CollisionConfig config) {
  return backend_->set_obstacle_collision_config(handle, config);
}

Result World::set_obstacle_position(uint32_t handle,
                                    ConstSpan<float> position) {
  return backend_->set_obstacle_position(handle, position);
}

}  // namespace silk
