#pragma once

#include <Eigen/Core>

#include "backend/cpu/ecs.hpp"
#include "backend/cpu/solver/barrier_constrain.hpp"

namespace silk::cpu {

/// @brief Reset cloth solver state to initial simulation conditions.
/// @param registry ECS registry storing cloth-related components.
/// @return void
void batch_reset_cloth_simulation(Registry& registry);

/// Prepare cloths for solver stepping at a given time step.
///  Ensures `ClothTopology`, `ClothSolverContext`, `ObjectState`, and
/// `CpuObjectCollider` exist and are valid.
///
///  @param registry ECS storage for all components.
///  @param entity ECS cloth entity.
///  @param dt       Time step in seconds
///  @param state_offset Object state offset.
///  @return false on factorization failure (e.g., non‑SPD system), true
///          otherwise.
bool prepare_cloth_simulation(Registry& registry, Entity& entity, float dt,
                              int state_offset);

/// Assemble step-invariant initial RHS for all cloth entities.
///
///  Writes each object's contribution into its state slice
///  [state_offset, state_offset + state_num).
///
///  @param registry Registry containing entities and components.
///  @param init_rhs Global RHS vector sized to the sum of state_num across
///  entities.
///  @pre rhs is sized to the sum of state_num across entities.
///  @post Overwrites corresponding segments of rhs.
void batch_compute_cloth_invariant_rhs(Registry& registry,
                                       Eigen::VectorXf& rhs);

/// Prepare all cloths for current solver outer loop.
///  Adds momentum and (optionally) barrier‑constraint terms into `rhs` and
///  updates the cached factorization to account for active barrier weights.
///
///  @param registry ECS storage.
///  @param global_state Vectorized current positions for all dofs.
///  @param global_state_velocity Vectorized velocities.
///  @param global_state_acceleration External acceleration per dof.
///  @param barrier_constrain Diagonal LHS weights and additive RHS.
///  @param rhs In/out right‑hand side to be accumulated into.
///  @return false if a per‑cloth update fails, true otherwise.
bool batch_compute_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs);

/// Solve all cloths' projective dynamics equation to produce a position update.
///  Projects in‑plane elastic constraints locally, then performs a global
///  linear solve using the cached Cholesky factorization (with barrier update
///  if active).
///
///  @param registry ECS storage.
///  @param global_state Vectorized current positions for all dofs.
///  @param outer_rhs Right‑hand side assembled by the outer loop.
///  @param solution Output solution vector; same layout/size as `global_state`.
///  @return false if the linear solve fails, true otherwise.
bool batch_compute_cloth_inner_loop(Registry& registry,
                                    const Eigen::VectorXf& global_state,
                                    const Eigen::VectorXf& outer_rhs,
                                    Eigen::VectorXf& solution);

}  // namespace silk::cpu
