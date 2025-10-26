/** @file
 * GPU solver pipeline - main GPU cloth physics solver.
 *
 * This is the GPU equivalent of CpuSolverPipeline, using CUDA-accelerated
 * Jacobi iteration instead of CPU CHOLMOD Cholesky factorization.
 */

#pragma once

#include <Eigen/Core>
#include <vector>

#include "collision/cpu/bbox.hpp"
#include "collision/cpu/collision.hpp"
#include "collision/cpu/pipeline.hpp"
#include "ecs.hpp"
#include "object_state.hpp"
#include "solver/cpu/barrier_constrain.hpp"

namespace silk {

/**
 * @brief GPU-accelerated cloth physics solver pipeline.
 *
 * Implements projective dynamics with GPU Jacobi iteration for the linear solve.
 * The outer/inner loop structure mirrors CpuSolverPipeline, but uses GPU
 * acceleration for solving the linear system Hx = b.
 *
 * Key differences from CPU solver:
 * - Uses GPU Jacobi iteration instead of CHOLMOD Cholesky
 * - Diagonal updates for barriers instead of factorization updates
 * - Persistent GPU memory across time steps for efficiency
 */
class GpuSolverPipeline {
 public:
  // Simulation parameters (same as CPU solver)
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;

  // Numerical epsilon used by various tolerances
  float eps = 1e-6f;

 private:
  // Collision pipeline (shared with CPU solver for now)
  CpuCollisionPipeline collision_pipeline;

 public:
  /**
   * @brief Remove all GPU solver components from entities and clear caches.
   */
  void clear(Registry& registry);

  /**
   * @brief Reset simulation to initial state.
   */
  void reset(Registry& registry);

  /**
   * @brief Advance the simulation by one time step using GPU acceleration.
   * @param registry ECS registry containing solver states and components
   * @return True on success, false if GPU solver reports failure or
   *         catastrophic numeric issues are detected
   * @post Writes back `ObjectState::curr_state` and `state_velocity` per entity
   */
  bool step(Registry& registry);

 private:
  /**
   * @brief Prepare all entities for simulation and collect solver state into
   *        global array.
   */
  bool init(Registry& registry, ObjectState& global_state);

  /**
   * @brief Compute bounding box of entire scene.
   */
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

}  // namespace silk
