#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>

using Verts = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajorBit>;
using Faces = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajorBit>;

class Mesh {
 public:
  Verts V;
  Faces F;

  Mesh(const Verts& verts, const Faces& faces);
  Mesh(float* verts, uint32_t vert_num, int* faces, uint32_t face_num);
  bool is_valid() const;
};

class ClothConfig {
 public:
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

enum class WorldResult : int {
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

class World {
  class WorldImpl;
  std::unique_ptr<WorldImpl> impl_;

 public:
  // cloth API
  [[nodiscard]] WorldResult add_cloth(ClothConfig config, Handle& handle);
  [[nodiscard]] WorldResult remove_cloth(const Handle& handle);
  [[nodiscard]] WorldResult update_cloth(ClothConfig config,
                                         const Handle& handle);

  // solver API
  Eigen::Vector3f& constant_acce_field();
  Eigen::Vector3f constant_acce_field() const;
  uint32_t& max_iterations();
  uint32_t max_iterations() const;
  uint32_t& thread_num();
  uint32_t thread_num() const;
  float get_dt() const;
  [[nodiscard]] WorldResult set_dt(float dt);
  uint32_t get_low_freq_mode_num() const;
  [[nodiscard]] WorldResult get_low_freq_mode_num(uint32_t num);

  void solver_reset();
  [[nodiscard]] WorldResult init_solver();
  [[nodiscard]] WorldResult step();

  // position API
  [[nodiscard]] WorldResult update_position_constrain(
      const Handle& handle, Eigen::Ref<const Eigen::VectorXf> positions);
  [[nodiscard]] WorldResult get_current_position(
      const Handle& handle, Eigen::Ref<Eigen::VectorXf> positions) const;
};
