#pragma once

#include <Eigen/Core>

#include "../barrier_constrain.hpp"
#include "../ecs.hpp"

namespace silk {

/** Reset runtime solver state for all cloths.
 *  Sets offsets to zero, copies current mesh vertex positions into
 *  `SolverState::curr_state`, zeroes velocities, and clears any active
 *  barrier constraint update on the factorization.
 */
void reset_all_cloth_for_solver(Registry& registry);

/** Prepare all cloths for solving at a given time step.
 *  Ensures `SolverState`, `ClothStaticSolverData`, and
 *  `ClothDynamicSolverData` exist and are initialized.
 *
 *  @param registry ECS storage for all components.
 *  @param dt       Time step in seconds; must be positive.
 *  @return false on factorization failure (e.g., non‑SPD system), true
 *          otherwise.
 */
bool init_all_cloth_for_solver(Registry& registry, float dt);

/** Prepare all cloths for current solver outer loop.
 *  Adds momentum and (optionally) barrier‑constraint terms into `rhs` and
 *  updates the cached factorization to account for active barrier weights.
 *
 *  @param registry ECS storage.
 *  @param global_state Vectorized current positions for all dofs.
 *  @param global_state_velocity Vectorized velocities.
 *  @param global_state_acceleration External acceleration per dof.
 *  @param barrier_constrain Diagonal LHS weights and additive RHS.
 *  @param rhs In/out right‑hand side to be accumulated into.
 *  @return false if a per‑cloth update fails, true otherwise.
 */
bool compute_all_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs);

/** Solve all cloths' projective dynamics equation to produce a position update.
 *  Projects in‑plane elastic constraints locally, then performs a global
 *  linear solve using the cached Cholesky factorization (with barrier update
 *  if active).
 *
 *  @param registry ECS storage.
 *  @param global_state Vectorized current positions for all dofs.
 *  @param init_rhs Right‑hand side assembled by the outer loop.
 *  @param solution Output solution vector; same layout/size as `global_state`.
 *  @return false if the linear solve fails, true otherwise.
 */
bool compute_all_cloth_inner_loop(Registry& registry,
                                  const Eigen::VectorXf& global_state,
                                  const Eigen::VectorXf& init_rhs,
                                  Eigen::VectorXf& solution);
}  // namespace silk
