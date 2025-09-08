#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace silk {

enum class Result {
  Success,
  InvalidConfig,
  TooManyBody,
  InvalidHandle,
  InvalidMesh,
  IncorrectPinNum,
  IncorrectPositionNum,
  CholeskyDecompositionFail,
  NeedInitSolverFirst,
  IterativeSolveFail
};

template <typename T>
struct Span {
  T* data = nullptr;
  int num = 0;
};

template <typename T>
struct ConstSpan {
  const T* data = nullptr;
  int num = 0;
};

struct MeshConfig {
  ConstSpan<float> verts;
  ConstSpan<int> faces;
};

struct CollisionConfig {
 public:
  bool is_collision_on = true;
  bool is_self_collision_on = true;
  int group = 0;
  float restitution = 0.3f;
  float friction = 0.3f;
};

struct ClothConfig {
  float elastic_stiffness = 100.0f;
  float bending_stiffness = 0.0001f;
  float density = 0.1f;
  float damping = 0.01f;
};

struct GlobalConfig {
  float acceleration_x = 0.0f;
  float acceleration_y = 0.0f;
  float acceleration_z = -10.0f;
  int max_outer_iteration = 100;
  int max_inner_iteration = 100;
  float dt = 1.0f / 60.0f;
};

std::string to_string(Result result);

class World {
  class WorldImpl;
  std::unique_ptr<WorldImpl> impl_;

 public:
  World();
  ~World();
  World(World&) = delete;
  World& operator=(World&) = delete;
  World(World&&);
  World& operator=(World&&);

  // Global API
  [[nodiscard]] Result set_global_config(GlobalConfig config);
  void clear();

  // Solver API
  [[nodiscard]] Result solver_init();
  [[nodiscard]] Result solver_step();
  [[nodiscard]] Result solver_reset();

  // cloth API
  [[nodiscard]] Result add_cloth(ClothConfig cloth_config,
                                 CollisionConfig collision_config,
                                 MeshConfig mesh_config,
                                 ConstSpan<int> pin_index, uint32_t& handle);
  [[nodiscard]] Result remove_cloth(uint32_t handle);
  [[nodiscard]] Result get_cloth_position(uint32_t handle,
                                          Span<float> position) const;
  [[nodiscard]] Result set_cloth_config(uint32_t handle, ClothConfig config);
  [[nodiscard]] Result set_cloth_collision_config(uint32_t handle,
                                                  CollisionConfig config);
  [[nodiscard]] Result set_cloth_pin_index(uint32_t handle,
                                           ConstSpan<int> pin_index);
  [[nodiscard]] Result set_cloth_pin_position(uint32_t handle,
                                              ConstSpan<float> position);

  // Obstacle API
  [[nodiscard]] Result add_obstacle(CollisionConfig collision_config,
                                    MeshConfig mesh_config, uint32_t& handle);
  [[nodiscard]] Result remove_obstacle(uint32_t handle);
  [[nodiscard]] Result set_obstacle_collision_config(uint32_t handle,
                                                     CollisionConfig config);
  [[nodiscard]] Result set_obstacle_position(uint32_t handle,
                                             ConstSpan<float> position);
};

}  // namespace silk
