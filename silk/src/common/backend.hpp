#pragma once

#include "silk/silk.hpp"

namespace silk {

class World::IBackend {
 public:
  virtual ~IBackend() = default;

  // Global API
  virtual Result set_global_config(GlobalConfig config) = 0;
  virtual void clear() = 0;

  // Solver API
  virtual Result solver_step() = 0;
  virtual Result solver_reset() = 0;

  // Cloth API
  virtual Result add_cloth(ClothConfig cloth_config,
                           CollisionConfig collision_config,
                           MeshConfig mesh_config, ConstSpan<int> pin_index,
                           uint32_t& handle) = 0;
  virtual Result remove_cloth(uint32_t handle) = 0;
  virtual Result get_cloth_position(uint32_t handle,
                                    Span<float> position) const = 0;
  virtual Result set_cloth_config(uint32_t handle, ClothConfig config) = 0;
  virtual Result set_cloth_collision_config(uint32_t handle,
                                            CollisionConfig config) = 0;
  virtual Result set_cloth_pin_index(uint32_t handle,
                                     ConstSpan<int> pin_index) = 0;
  virtual Result set_cloth_pin_position(uint32_t handle,
                                        ConstSpan<float> position) = 0;

  // Obstacle API
  virtual Result add_obstacle(CollisionConfig collision_config,
                              MeshConfig mesh_config, uint32_t& handle) = 0;
  virtual Result remove_obstacle(uint32_t handle) = 0;
  virtual Result set_obstacle_collision_config(uint32_t handle,
                                               CollisionConfig config) = 0;
  virtual Result set_obstacle_position(uint32_t handle,
                                       ConstSpan<float> position) = 0;
};

}  // namespace silk
