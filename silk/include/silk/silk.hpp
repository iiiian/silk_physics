#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>

namespace silk {

using Verts = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajorBit>;
using Faces = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajorBit>;

class Mesh {
 public:
  Verts V;
  Faces F;

  Mesh() = default;
  Mesh(const Verts& verts, const Faces& faces);
  Mesh(float* verts, uint32_t vert_num, int* faces, uint32_t face_num);

  bool is_valid() const;
};

class ClothConfig {
 public:
  bool enable_collision = true;
  bool enable_self_collision = true;
  float elastic_stiffness = 1.0f;
  float bending_stiffness = 1.0f;
  float density = 1.0f;

  Mesh mesh;
  Eigen::VectorXi pinned_verts;

  bool is_valid() const;
};

enum class HandleType { Cloth, RigidBody, SoftBody, Hair, Collider };

struct Handle {
  HandleType type;
  uint32_t value;
};

enum class WorldResult {
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

std::string to_string(WorldResult result);

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
  [[nodiscard]] WorldResult add_cloth(ClothConfig config, Handle& handle);
  [[nodiscard]] WorldResult remove_cloth(const Handle& handle);
  [[nodiscard]] WorldResult update_cloth(ClothConfig config,
                                         const Handle& handle);

  // solver API

  Eigen::Vector3f get_constant_acce_field() const;
  void set_constant_acce_field(Eigen::Vector3f acce);

  uint32_t get_max_iterations() const;
  void set_max_iterations(uint32_t iter);

  uint32_t get_thread_num() const;
  void set_thread_num(uint32_t num);

  float get_dt() const;
  [[nodiscard]] WorldResult set_dt(float dt);

  uint32_t get_low_freq_mode_num() const;
  [[nodiscard]] WorldResult set_low_freq_mode_num(uint32_t num);

  void solver_reset();
  [[nodiscard]] WorldResult solver_init();
  [[nodiscard]] WorldResult step();

  // position API
  [[nodiscard]] WorldResult update_position_constrain(
      const Handle& handle, Eigen::Ref<const Eigen::VectorXf> positions);
  [[nodiscard]] WorldResult get_current_position(
      const Handle& handle, Eigen::Ref<Eigen::VectorXf> positions) const;
};

}  // namespace silk
