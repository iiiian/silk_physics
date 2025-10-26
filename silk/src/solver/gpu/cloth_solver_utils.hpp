/** @file
 * GPU cloth solver utilities - GPU equivalent of CPU cloth solver utils.
 *
 * Provides batch operations for preparing and updating cloth simulations
 * using GPU Jacobi iteration instead of CPU CHOLMOD factorization.
 */

#pragma once

#include <Eigen/Core>

#include "ecs.hpp"
#include "solver/cpu/barrier_constrain.hpp"

namespace silk {

/**
 * @brief Reset all cloth solver states to initial simulation conditions.
 * @param registry ECS registry storing cloth-related components.
 */
void batch_reset_gpu_cloth_simulation(Registry& registry);

/**
 * @brief Prepare cloth entity for GPU solver stepping at given time step.
 *
 * Ensures `ClothTopology`, `GpuClothSolverContext`, `ObjectState`, and
 * collision components exist and are valid.
 *
 * @param registry ECS storage for all components
 * @param entity Cloth entity to prepare
 * @param dt Time step in seconds
 * @param state_offset Offset in global state array
 * @return false on GPU initialization failure, true otherwise
 */
bool prepare_gpu_cloth_simulation(Registry& registry, Entity& entity, float dt,
                                  int state_offset);

/**
 * @brief Assemble step-invariant initial RHS for all GPU cloth entities.
 *
 * Writes each object's contribution into its state slice
 * [state_offset, state_offset + state_num).
 *
 * @param registry Registry containing entities and components
 * @param init_rhs Global RHS vector sized to sum of state_num
 * @pre rhs is sized to the sum of state_num across entities
 * @post Overwrites corresponding segments of rhs
 */
void batch_compute_gpu_cloth_invariant_rhs(Registry& registry,
                                           Eigen::VectorXf& rhs);

/**
 * @brief Prepare all GPU cloths for current solver outer loop.
 *
 * Adds momentum and (optionally) barrier-constraint terms into `rhs` and
 * updates the diagonal for active barrier weights.
 *
 * @param registry ECS storage
 * @param global_state Vectorized current positions for all dofs
 * @param global_state_velocity Vectorized velocities
 * @param global_state_acceleration External acceleration per dof
 * @param barrier_constrain Diagonal LHS weights and additive RHS
 * @param rhs In/out right-hand side to be accumulated into
 * @return false if a per-cloth update fails, true otherwise
 */
bool batch_compute_gpu_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs);

/**
 * @brief Solve all GPU cloths' projective dynamics equation using Jacobi iteration.
 *
 * Projects in-plane elastic constraints locally, then performs GPU Jacobi
 * iteration using the prepared diagonal and RHS.
 *
 * @param registry ECS storage
 * @param global_state Vectorized current positions for all dofs
 * @param outer_rhs Right-hand side assembled by the outer loop
 * @param solution Output solution vector; same layout/size as `global_state`
 * @return false if the GPU solve fails, true otherwise
 */
bool batch_compute_gpu_cloth_inner_loop(Registry& registry,
                                        const Eigen::VectorXf& global_state,
                                        const Eigen::VectorXf& outer_rhs,
                                        Eigen::VectorXf& solution);

}  // namespace silk
