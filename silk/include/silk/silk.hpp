#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace silk {

enum class Result {
  Success,
  InvalidTimeStep,
  InvalidLowFreqModeNum,
  InvalidConfig,
  TooManyBody,
  InvalidHandle,
  IncorrectPositionConstrainLength,
  IncorrentOutputPositionLength,
  EigenDecompositionfail,
  IterativeSolverInitFail,
  NeedInitSolverBeforeSolve,
  IterativeSolveFail
};

class MeshConfig {
 public:
  float* vertices;
  int vert_num;
  int* faces;
  int face_num;

  Result validate() const;
};

class PininingConfig {
 public:
  int* pinned_vertices;
  int pinned_num;

  Result validate() const;
};

class CollisionConfig {
 public:
  bool is_collision_on = true;
  bool is_self_collision_on = true;
  int group = 0;
  float damping = 0.0f;
  float friction = 0.0f;

  Result validate() const;
};

class ClothConfig {
 public:
  float elastic_stiffness = 1.0f;
  float bending_stiffness = 1.0f;
  float density = 1.0f;

  Result validate() const;
};

struct ClothHandle {
  uint32_t value;
};

struct ObstacleHandle {
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

  // cloth API
  [[nodiscard]] Result add_cloth(ClothConfig cloth_config,
                                 CollisionConfig collision_config,
                                 MeshConfig mesh_config,
                                 PininingConfig pinning_config,
                                 ClothHandle& handle);
  [[nodiscard]] Result remove_cloth(const ClothHandle& handle);
  // update cloth config
  [[nodiscard]] Result update_cloth(const ClothHandle& handle,
                                    ClothConfig config);
  // update cloth collision config
  [[nodiscard]] Result update_cloth(const ClothHandle& handle,
                                    CollisionConfig config);
  // update cloth mesh and pinning config
  [[nodiscard]] Result update_cloth(const ClothHandle& handle,
                                    MeshConfig mesh_config,
                                    PininingConfig pining_config);
  [[nodiscard]] Result set_cloth_pinned(const ClothHandle& handle,
                                        float* pinned_vertices, int pinned_num);
  [[nodiscard]] Result get_cloth_state(const ClothHandle& handle,
                                       float* position, int num) const;

  // obstacle API
  [[nodiscard]] Result add_obstacle(CollisionConfig collision_config,
                                    MeshConfig mesh_config,
                                    ObstacleHandle& handle);
  [[nodiscard]] Result remove_obstacle(const ObstacleHandle& handle);
  // update obstacle collision config
  [[nodiscard]] Result update_obstacle(const ObstacleHandle& handle,
                                       CollisionConfig config);
  // update obstacle mesh
  [[nodiscard]] Result update_obstacle(const ObstacleHandle& handle,
                                       MeshConfig mesh_config);

  // solver API

  void get_acceleration(float& x_acceleration, float& y_acceleration,
                        float& z_acceleration) const;
  void set_acceleration(float x_acceleration, float y_acceleration,
                        float z_acceleration);

  int get_max_iteration() const;
  void set_max_iterations(int iter);

  int get_thread_num() const;
  void set_thread_num(int num);

  float get_dt() const;
  [[nodiscard]] Result set_dt(float dt);

  int get_low_freq_mode_num() const;
  [[nodiscard]] Result set_low_freq_mode_num(int num);

  void solver_reset();
  [[nodiscard]] Result solver_init();
  [[nodiscard]] Result step();
};

}  // namespace silk
