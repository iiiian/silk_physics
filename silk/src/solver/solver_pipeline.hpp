#pragma once

#include <Eigen/Core>
#include <vector>

#include "../barrier_constrain.hpp"
#include "../bbox.hpp"
#include "../collision.hpp"
#include "../collision_pipeline.hpp"
#include "../ecs.hpp"
#include "../object_state.hpp"

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
  /** @brief Remove all solver components from entities and clear caches.
   */
  void clear(Registry& registry);

  /** @brief Reset simulation to initial state.
   */
  void reset(Registry& registry);

  /**
   * @brief Advance the simulation by one time step.
   * @param registry ECS registry containing solver states and components.
   * @param collision_pipeline Collision detector used for CCD queries.
   * @returns True on success, false if a sub-solver reports failure or
   *          catastrophic numeric issues are detected.
   * @post Writes back `ObjectState::curr_state` and `state_velocity` per
   * entity.
   */
  bool step(Registry& registry, CollisionPipeline& collision_pipeline);

 private:
  /** @brief Assemble a single global solver state across all entities.
   */
  ObjectState compute_global_state(Registry& registry);

  Bbox compute_scene_bbox(const Eigen::VectorXf& state);

  /** @brief Drop collisions whose entities are no longer alive in the registry.
   */
  void cleanup_collisions(Registry& registry);

  /** @brief Build diagonal LHS weights and RHS targets from current collisions.
   */
  BarrierConstrain compute_barrier_constrain(const Eigen::VectorXf& state);

  /** @brief Project state onto barrier targets where constraints are active.
   */
  void enforce_barrier_constrain(const BarrierConstrain& barrier_constrain,
                                 const Bbox& scene_bbox,
                                 Eigen::VectorXf& state) const;
};

}  // namespace silk
