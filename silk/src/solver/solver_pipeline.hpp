#pragma once

#include <Eigen/Core>
#include <vector>

#include "../barrier_constrain.hpp"
#include "../bbox.hpp"
#include "../collision.hpp"
#include "../collision_pipeline.hpp"
#include "../ecs.hpp"
#include "../solver_state.hpp"

namespace silk {

class SolverPipeline {
 public:
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;

  // Numerical epsilon used by various tolerances.
  float eps = 1e-6f;

 private:
  std::vector<Collision> collisions_;

 public:
  /**
   * @brief Remove all solver components from entities and clear caches.
   */
  void clear(Registry& registry);

  /**
   * @brief Reset solver-related data and clear cached collisions.
   * @details Keeps entities/components but reinitializes their solver data to a
   *          clean state, suitable for starting a new solve.
   */
  void reset(Registry& registry);

  /**
   * @brief Initialize all cloth data needed by the solver.
   * @returns True on success, false if any cloth init fails.
   */
  bool init(Registry& registry);

  /**
   * @brief Advance the simulation by one time step.
   * @param registry ECS registry containing solver states and components.
   * @param collision_pipeline Collision detector used for CCD queries.
   * @returns True on success, false if a sub-solver reports failure or
   *          catastrophic numeric issues are detected.
   * @post Writes back `SolverState::curr_state` and `state_velocity` per
   * entity.
   */
  bool step(Registry& registry, CollisionPipeline& collision_pipeline);

 private:
  /**
   * @brief Assemble a single global solver state across all entities.
   * @details Also applies per-entity velocity damping as
   *          `state_velocity = (1 - damping) * state_velocity`.
   */
  SolverState compute_global_state(Registry& registry);

  Bbox compute_scene_bbox(const Eigen::VectorXf& state);

  /** @brief Drop collisions whose entities are no longer alive in the registry.
   */
  void cleanup_collisions(Registry& registry);

  /**
   * @brief Build diagonal LHS weights and RHS targets from current collisions.
   * @details For each active DOF that participates in a collision,
   *          collision stiffness is accumulated into `lhs`, and a reflected
   *          target position is accumulated into `rhs`.
   */
  BarrierConstrain compute_barrier_constrain(const Eigen::VectorXf& state);

  /**
   * @brief Project state onto barrier targets where constraints are active.
   * @details For entries with positive `lhs`, replaces `state[i]` by
   *          `rhs[i] / lhs[i]`.
   */
  void enforce_barrier_constrain(const BarrierConstrain& barrier_constrain,
                                 const Bbox& scene_bbox,
                                 Eigen::VectorXf& state) const;
};

}  // namespace silk
