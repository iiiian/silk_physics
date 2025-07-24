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
struct Span {
  T* data;
  int num;
};

template <typename T>
struct ConstSpan {
  const T* data;
  int num;
};

struct MeshConfig {
  ConstSpan<float> verts;
  ConstSpan<int> faces;
  ConstSpan<int> pin_index;
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
                                 MeshConfig mesh_config, Cloth& cloth);
  [[nodiscard]] Result remove_cloth(Cloth cloth);
  [[nodiscard]] Result get_cloth_position(Cloth cloth,
                                          Span<float> position) const;
  [[nodiscard]] Result set_cloth_config(Cloth cloth, ClothConfig config);
  [[nodiscard]] Result set_cloth_collision_config(Cloth cloth,
                                                  CollisionConfig config);
  [[nodiscard]] Result set_cloth_pin_index(Cloth cloth, MeshConfig mesh_config);
  [[nodiscard]] Result set_cloth_pin_position(Cloth cloth,
                                              ConstSpan<float> position);

  // Obstacle API
  [[nodiscard]] Result add_obstacle(CollisionConfig collision_config,
                                    MeshConfig mesh_config, Obstacle& obstacle);
  [[nodiscard]] Result remove_obstacle(Obstacle obstacle);
  [[nodiscard]] Result set_obstacle_collision_config(Obstacle obstacle,
                                                     CollisionConfig config);
  [[nodiscard]] Result set_obstacle_position(Obstacle obstacle,
                                             ConstSpan<float> position);
};

}  // namespace silk
