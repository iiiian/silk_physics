#pragma once

#include "common/backend.hpp"
#include "silk/silk.hpp"

namespace silk::cpu {

class CpuBackend : public World::IBackend {
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

 public:
  CpuBackend();
  ~CpuBackend() override;
  CpuBackend(CpuBackend&&) noexcept = default;
  CpuBackend& operator=(CpuBackend&&) noexcept = default;
  CpuBackend(const CpuBackend&) = delete;
  CpuBackend& operator=(const CpuBackend&) = delete;

  // Global API
  Result set_global_config(GlobalConfig config) override;
  void clear() override;

  // Solver API
  Result solver_step() override;
  Result solver_reset() override;

  // Cloth API
  Result add_cloth(ClothConfig cloth_config, CollisionConfig collision_config,
                   MeshConfig mesh_config, ConstSpan<int> pin_index,
                   uint32_t& handle) override;
  Result remove_cloth(uint32_t handle) override;
  Result get_cloth_position(uint32_t handle,
                            Span<float> position) const override;
  Result set_cloth_config(uint32_t handle, ClothConfig config) override;
  Result set_cloth_collision_config(uint32_t handle,
                                    CollisionConfig config) override;
  Result set_cloth_pin_index(uint32_t handle,
                             ConstSpan<int> pin_index) override;
  Result set_cloth_pin_position(uint32_t handle,
                                ConstSpan<float> position) override;

  // Obstacle API
  Result add_obstacle(CollisionConfig collision_config, MeshConfig mesh_config,
                      uint32_t& handle) override;
  Result remove_obstacle(uint32_t handle) override;
  Result set_obstacle_collision_config(uint32_t handle,
                                       CollisionConfig config) override;
  Result set_obstacle_position(uint32_t handle,
                               ConstSpan<float> position) override;
};

}  // namespace silk::cpu
