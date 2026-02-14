/// @file silk.hpp
/// @brief Core API for Silk physics simulation library.
///
/// Silk is a cloth physics library built on projective dynamics with continuous
/// collision detection. Provides cloth simulation, obstacle interaction, and
/// pin constraints through an entity-component system architecture.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "silk/result.hpp"

namespace silk {

/// Backend selection for the simulation world.
enum class Backend { Cpu, Gpu };

// Internal backend interface (defined in source tree)
class IBackend;

/// @brief Non-owning view over contiguous array data.
/// @tparam T Element type
template <typename T>
class Span {
 public:
  T* data = nullptr;  ///< Pointer to first element (may be null if size is 0)
  int size = 0;       ///< Number of elements

 public:
  Span() = default;
  Span(T* ptr, int size) : data(ptr), size(size) {}
  Span(std::vector<T>& vec) : data(vec.data()), size(vec.size()) {}
};

/// @brief Non-owning read-only view over contiguous array data.
/// @tparam T Element type
template <typename T>
class ConstSpan {
 public:
  const T* data =
      nullptr;   ///< Pointer to first element (may be null if size is 0)
  int size = 0;  ///< Number of elements

 public:
  ConstSpan() = default;
  ConstSpan(const T* ptr, int size) : data(ptr), size(size) {}
  ConstSpan(const std::vector<T>& vec) : data(vec.data()), size(vec.size()) {}
};

/// @brief Triangle mesh definition for cloth and obstacles.
struct MeshConfig {
  ConstSpan<float> verts;  ///< Vertex positions as [x0,y0,z0, x1,y1,z1, ...]
  ConstSpan<int> faces;    ///< Triangle indices as [i0,j0,k0, i1,j1,k1, ...]
};

/// @brief Collision behavior configuration for entities.
struct CollisionConfig {
 public:
  bool is_collision_on = true;       ///< Enable collision
  bool is_self_collision_on = true;  ///< Enable self-collision
  int group = 0;                     ///< Collision group
  float restitution = 0.3f;          ///< Coefficient of restitution [0,1]
  float friction = 0.3f;             ///< Coefficient of friction [0,1]
};

/// @brief Physical properties for cloth simulation.
struct ClothConfig {
  /// In-plane stretching resistance (higher = stiffer)
  float elastic_stiffness = 100.0f;
  /// Out-of-plane bending resistance (higher = stiffer)
  float bending_stiffness = 0.0001f;
  /// Mass density per unit area
  float density = 0.1f;
  /// Velocity damping factor [0,1] (higher = more damping)
  float damping = 0.01f;
};

/// @brief Solver backend selection
enum class SolverBackend {
  CPU,   ///< Use CPU solver (TBB parallel, default)
  GPU,   ///< Use GPU solver (CUDA acceleration)
  Auto   ///< Automatically select based on availability and mesh size
};

/// @brief Global simulation parameters.
struct GlobalConfig {
  /// Constant acceleration in X direction
  float acceleration_x = 0.0f;
  /// Constant acceleration in Y direction
  float acceleration_y = 0.0f;
  /// Constant acceleration in Z direction
  float acceleration_z = -10.0f;

  /// Maximum collision substeps per frame
  int max_outer_iteration = 100;
  /// Maximum projective dynamics iterations per substep
  int max_inner_iteration = 100;

  /// Time step size in seconds
  float dt = 1.0f / 60.0f;

  /// Solver backend selection (CPU, GPU, or Auto)
  SolverBackend solver_backend = SolverBackend::CPU;
};

/// @brief Main simulation world managing all physics entities and systems.
class World {
 public:
  class IBackend;

 private:
  std::unique_ptr<IBackend> backend_;

 public:
  World();
  World(const World&) = delete;
  World(World&&);

  ~World();

  World& operator=(const World&) = delete;
  World& operator=(World&&);

  // ---------------------------------------
  // Global API
  // ---------------------------------------

  /// @brief Select computation backend (CPU or GPU).
  /// Defaults to CPU. Switching clears current simulation state.
  /// @return Success or NoCudaSupport if GPU backend is unavailable.
  [[nodiscard]] Result set_backend(Backend backend);

  /// @brief Configure global simulation parameters.
  /// @param config Global physics settings (gravity, timestep, solver limits)
  /// @return Success or failure result
  [[nodiscard]] Result set_global_config(GlobalConfig config);

  /// @brief Remove all objects and simulation data.
  void clear();

  // Solver API

  /// @brief Advance simulation by one timestep.
  ///
  /// @return Success, or CholeskyDecompositionFail if numerical issues occur
  [[nodiscard]] Result solver_step();

  /// @brief Reset simulation state.
  /// @return Success result
  [[nodiscard]] Result solver_reset();

  // Cloth API

  /// @brief Create a new cloth object in the simulation.
  ///
  /// Validates mesh topology. Cloth mesh must be
  /// manifold, single-component, and have reasonable triangle quality.
  ///
  /// @param cloth_config Physical properties
  /// @param collision_config Collision settings
  /// @param mesh_config Triangle mesh definition
  /// @param pin_index Indices of vertices to pin in place (may be empty)
  /// @param handle Output handle for the new cloth object. Set to zero if fails.
  /// @return Success with valid handle, or error (InvalidMesh, TooManyBody)
  [[nodiscard]] Result add_cloth(ClothConfig cloth_config,
                                 CollisionConfig collision_config,
                                 MeshConfig mesh_config,
                                 ConstSpan<int> pin_index, uint32_t& handle);

  /// @brief Remove cloth object from simulation.
  /// @param handle Cloth object handle
  /// @return Success, or InvalidHandle if handle is invalid
  [[nodiscard]] Result remove_cloth(uint32_t handle);

  /// @brief Retrieve current vertex positions of cloth object.
  ///
  /// Positions are returned in same layout as original mesh: [x0,y0,z0,
  /// x1,y1,z1, ...]. Buffer size must exactly match 3 * vertex_count.
  ///
  /// @param handle Cloth object handle
  /// @param position Output buffer for vertex positions (must be pre-allocated)
  /// @return Success, or InvalidHandle/IncorrectPositionNum on error
  [[nodiscard]] Result get_cloth_position(uint32_t handle,
                                          Span<float> position) const;

  /// @brief Update physical properties of existing cloth.
  ///
  /// Triggers solver reinitialization, so avoid calling every frame.
  ///
  /// @param handle Cloth object handle
  /// @param config New physical properties
  /// @return Success, or InvalidHandle if cloth not found
  [[nodiscard]] Result set_cloth_config(uint32_t handle, ClothConfig config);

  /// @brief Update collision behavior of existing cloth.
  /// @param handle Cloth object handle
  /// @param config New collision settings
  /// @return Success, or InvalidHandle if cloth not found
  [[nodiscard]] Result set_cloth_collision_config(uint32_t handle,
                                                  CollisionConfig config);

  /// @brief Update which vertices are pinned in place.
  ///
  /// Triggers solver reinitialization, so avoid calling every frame.
  ///
  /// @param handle Cloth object handle
  /// @param pin_index Vertex indices to pin (may be empty to unpin all)
  /// @return Success, or InvalidHandle if cloth not found
  [[nodiscard]] Result set_cloth_pin_index(uint32_t handle,
                                           ConstSpan<int> pin_index);

  /// @brief Move pinned vertices to new target positions.
  ///
  /// Position count must match 3 * pin_count.
  ///
  /// @param handle Cloth object handle
  /// @param position Target positions for pinned vertices [x0,y0,z0, x1,y1,z1,
  /// ...]
  /// @return Success, or InvalidHandle/IncorrectPinNum on error
  [[nodiscard]] Result set_cloth_pin_position(uint32_t handle,
                                              ConstSpan<float> position);

  // ---------------------------------------
  // Obstacle API
  // ---------------------------------------

  /// @brief Create a obstacle for collision.
  ///
  /// Obstacles do not simulate physics but participate in collision
  /// detection. Can be moved via set_obstacle_position().
  ///
  /// @param collision_config Collision behavior
  /// @param mesh_config Triangle mesh definition
  /// @param handle Output handle for the new obstacle. Set to zero if fails.
  /// @return Success with valid handle, or error (InvalidMesh, TooManyBody)
  [[nodiscard]] Result add_obstacle(CollisionConfig collision_config,
                                    MeshConfig mesh_config, uint32_t& handle);

  /// @brief Remove obstacle from simulation.
  /// @param handle Obstacle handle
  /// @return Success, or InvalidHandle if handle is invalid
  [[nodiscard]] Result remove_obstacle(uint32_t handle);

  /// @brief Update collision properties of existing obstacle.
  /// @param handle Obstacle handle
  /// @param config New collision settings
  /// @return Success, or InvalidHandle if obstacle not found
  [[nodiscard]] Result set_obstacle_collision_config(uint32_t handle,
                                                     CollisionConfig config);

  /// @brief Move obstacle to new position.
  ///
  /// Positions should match original mesh layout. Obstacle moves smoothly
  /// between old and new positions over the timestep.
  ///
  /// @param handle Obstacle handle
  /// @param position New vertex positions [x0,y0,z0, x1,y1,z1, ...]
  /// @return Success, or InvalidHandle/IncorrectPositionNum on error
  [[nodiscard]] Result set_obstacle_position(uint32_t handle,
                                             ConstSpan<float> position);
};

}  // namespace silk
