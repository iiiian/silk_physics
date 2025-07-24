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
  IncorrectPinNum,
  IncorrectPositionNum,
  EigenDecompositionFail,
  NeedInitSolverFirst,
  IterativeSolveFail
};

template <typename T>
struct View {
  T* data;
  int num;
};

template <typename T>
struct ConstView {
  const T* data;
  int num;
};

struct MeshConfig {
  ConstView<float> verts;
  ConstView<int> faces;
  ConstView<int> pin_index;
};

struct CollisionConfig {
 public:
  bool is_collision_on;
  bool is_self_collision_on;
  int group;
  float damping;
  float friction;
};

struct ClothConfig {
  float elastic_stiffness = 1.0f;
  float bending_stiffness = 1.0f;
  float density = 1.0f;
};

struct GlobalConfig {
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  int max_iteration;
  float r;
  float dt;
  float ccd_walkback;
  float toi_tolerance;
  float toi_refine_iteration;
  float eps;
};

struct Cloth {
  uint32_t value;
};

struct Obstacle {
  uint32_t value;
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
                                 MeshConfig mesh_config, Cloth& handle);
  [[nodiscard]] Result remove_cloth(Cloth handle);
  [[nodiscard]] Result get_cloth_position(Cloth handle,
                                          View<float> position) const;
  [[nodiscard]] Result set_cloth_config(Cloth handle, ClothConfig config);
  [[nodiscard]] Result set_cloth_collision_config(Cloth handle,
                                                  CollisionConfig config);
  [[nodiscard]] Result set_cloth_mesh(Cloth handle, MeshConfig mesh_config);
  [[nodiscard]] Result set_cloth_pin(Cloth handle, ConstView<float> pin);

  // Obstacle API
  [[nodiscard]] Result add_obstacle(CollisionConfig collision_config,
                                    MeshConfig mesh_config, Obstacle& handle);
  [[nodiscard]] Result remove_obstacle(Obstacle handle);
  [[nodiscard]] Result set_obstacle_collision_config(Obstacle handle,
                                                     CollisionConfig config);
  [[nodiscard]] Result set_obstacle_mesh(const Obstacle& handle,
                                         MeshConfig mesh_config);
  [[nodiscard]] Result set_obstacle_position(Cloth handle,
                                             ConstView<float> position);
};

}  // namespace silk
