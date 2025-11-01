#pragma once

/**
 * @file pipeline.hpp (GPU version)
 * @brief GPU-accelerated solver pipeline
 *
 * This pipeline uses GPU kernels for the elastic RHS computation while keeping
 * the linear solve, collision detection, and outer loop on the CPU.
 */

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "collision/cpu/bbox.hpp"
#include "collision/cpu/collision.hpp"
#include "collision/cpu/pipeline.hpp"
#include "ecs.hpp"
#include "object_state.hpp"
#include "solver/cpu/barrier_constrain.hpp"
#include "solver/gpu/gpu_cloth_solver_context.hpp"

namespace silk {
namespace gpu {

/**
 * @brief GPU-accelerated solver pipeline
 *
 * This class provides the same interface as CpuSolverPipeline but uses GPU
 * kernels for the computationally intensive elastic RHS computation.
 *
 * Workflow:
 * 1. Outer loop (CPU): momentum, barriers, collision detection
 * 2. Inner loop:
 *    a. Elastic RHS projection (GPU) ← ACCELERATED
 *    b. Linear solve (CPU, CHOLMOD)
 * 3. Update state and velocity (CPU)
 *
 * The GPU is used for the per-face SVD projection and force accumulation,
 * while the sparse linear solve remains on CPU using CHOLMOD.
 */
class GpuSolverPipeline {
 public:
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;

  // Numerical epsilon used by various tolerances.
  float eps = 1e-6f;

 private:
  CpuCollisionPipeline collision_pipeline;

  // GPU contexts for each cloth entity (indexed by entity ID)
  std::vector<std::unique_ptr<GpuClothSolverContext>> gpu_contexts_;

 public:
  /**
   * @brief Remove all solver components from entities and clear caches.
   */
  void clear(Registry& registry);

  /**
   * @brief Reset simulation to initial state.
   */
  void reset(Registry& registry);

  /**
   * @brief Advance the simulation by one time step using GPU acceleration.
   * @param registry ECS registry containing solver states and components.
   * @return True on success, false if a sub-solver reports failure or
   *         catastrophic numeric issues are detected.
   * @post Writes back `ObjectState::curr_state` and `state_velocity` per
   * entity.
   */
  bool step(Registry& registry);

 private:
  /**
   * @brief Prepare all entities for simulation and create GPU contexts.
   */
  bool init(Registry& registry, ObjectState& global_state);

  /**
   * @brief Initialize GPU context for a cloth entity
   */
  bool init_gpu_context_for_entity(Entity& entity, Registry& registry);

  Bbox compute_scene_bbox(const Eigen::VectorXf& state);

  /**
   * @brief Build diagonal LHS weights and RHS targets from current collisions.
   */
  BarrierConstrain compute_barrier_constrain(
      const Eigen::VectorXf& state, const std::vector<Collision>& collisions);

  /**
   * @brief Project state onto barrier targets where constraints are active.
   */
  void enforce_barrier_constrain(const BarrierConstrain& barrier_constrain,
                                 const Bbox& scene_bbox,
                                 Eigen::VectorXf& state) const;
};

}  // namespace gpu
}  // namespace silk
