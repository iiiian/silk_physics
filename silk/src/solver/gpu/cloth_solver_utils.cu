/** @file
 * GPU cloth solver utilities implementation.
 */

// Prevent NVCC from including x86 intrinsic headers that cause compilation errors
#ifdef __CUDACC__
#define _AMXTILEINTRIN_H_INCLUDED
#define _AMXBF16INTRIN_H_INCLUDED
#define _AMXINT8INTRIN_H_INCLUDED
#define _AMXFP16INTRIN_H_INCLUDED
#endif

#include "solver/gpu/cloth_solver_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <cassert>
#include <optional>
#include <unsupported/Eigen/KroneckerProduct>

#include "cloth_topology.hpp"
#include "collision/cpu/object_collider.hpp"
#include "logger.hpp"
#include "mesh.hpp"
#include "object_state.hpp"
#include "pin.hpp"
#include "solver/cpu/barrier_constrain.hpp"
#include "solver/gpu/cloth_solver_context.hpp"

namespace silk {

void batch_reset_gpu_cloth_simulation(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto mesh = registry.get<TriMesh>(e);
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<GpuClothSolverContext>(e);

    if (mesh && state && context) {
      state->state_offset = 0;
      state->curr_state = mesh->V.reshaped<Eigen::RowMajor>();
      state->state_velocity.setZero();

      // Reset barrier constraint flag
      context->has_barrier_constrain = false;
      context->HB_diag = context->H_diag;
    }
  }
}

bool prepare_gpu_cloth_simulation(Registry& registry, Entity& entity, float dt,
                                  int state_offset) {
  auto& e = entity;

  auto cloth_config = registry.get<ClothConfig>(e);
  auto collision_config = registry.get<CollisionConfig>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin = registry.get<Pin>(e);

  // Cloth entity sanity check
  assert(cloth_config && collision_config && mesh && pin);

  // Initialize or update ObjectState
  auto state = registry.get<ObjectState>(e);
  if (!state) {
    ObjectState new_state;
    new_state.state_offset = state_offset;
    new_state.state_num = 3 * mesh->V.rows();
    new_state.curr_state = mesh->V.reshaped<Eigen::RowMajor>();
    new_state.state_velocity = Eigen::VectorXf::Zero(new_state.state_num);
    state = registry.set<ObjectState>(e, std::move(new_state));
  } else {
    state->state_offset = state_offset;
  }
  assert(state != nullptr);

  // Initialize ClothTopology if needed
  auto topology = registry.get<ClothTopology>(e);
  if (!topology) {
    topology =
        registry.set<ClothTopology>(e, ClothTopology(*cloth_config, *mesh));
  }
  assert(topology != nullptr);

  // Initialize or update GpuClothSolverContext
  auto context = registry.get<GpuClothSolverContext>(e);
  if (context && context->dt == dt) {
    // Reuse existing context, just reset barrier flag
    context->has_barrier_constrain = false;
  } else {
    // Create new context (this initializes GPU memory)
    auto new_context = GpuClothSolverContext::make_cloth_solver_context(
        *cloth_config, *topology, *pin, dt);
    if (!new_context) {
      SPDLOG_ERROR("Failed to create GPU cloth solver context");
      return false;
    }
    context = registry.set<GpuClothSolverContext>(e, std::move(*new_context));
  }
  assert(context != nullptr);

  // Initialize or update collider
  auto collider = registry.get<CpuObjectCollider>(e);
  if (!collider) {
    auto new_collider = CpuObjectCollider(e.self, *collision_config, *mesh,
                                          *pin, context->mass, state_offset);
    collider = registry.set<CpuObjectCollider>(e, std::move(new_collider));
  } else {
    collider->state_offset = state_offset;
  }
  assert(collider != nullptr);

  return true;
}

/**
 * @brief Compute step-invariant RHS for a single GPU cloth.
 */
void compute_gpu_cloth_invariant_rhs(const GpuClothSolverContext& solver_context,
                                     const Pin& pin,
                                     Eigen::Ref<Eigen::VectorXf> rhs) {
  rhs.setZero();

  // Pin contribution
  for (int i = 0; i < pin.index.size(); ++i) {
    rhs(Eigen::seqN(3 * pin.index(i), 3)) =
        pin.pin_stiffness * pin.position(Eigen::seqN(3 * i, 3));
  }

  // Rest curvature contribution
  rhs += solver_context.C0;
}

void batch_compute_gpu_cloth_invariant_rhs(Registry& registry,
                                           Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<GpuClothSolverContext>(e);
    auto pin = registry.get<Pin>(e);

    if (state && context && pin) {
      auto seq = Eigen::seqN(state->state_offset, state->state_num);
      compute_gpu_cloth_invariant_rhs(*context, *pin, rhs(seq));
    }
  }
}

/**
 * @brief Outer loop for single GPU cloth - update diagonal and RHS with barriers.
 */
bool compute_gpu_cloth_outer_loop(
    Eigen::Ref<const Eigen::VectorXf> state,
    Eigen::Ref<const Eigen::VectorXf> state_velocity,
    Eigen::Ref<const Eigen::VectorXf> state_acceleration,
    Eigen::Ref<const Eigen::VectorXf> barrier_lhs,
    Eigen::Ref<const Eigen::VectorXf> barrier_rhs,
    GpuClothSolverContext& solver_context, Eigen::Ref<Eigen::VectorXf> rhs) {
  auto& s = solver_context;

  // Momentum contribution to RHS
  rhs.noalias() += (s.mass / (s.dt * s.dt)).asDiagonal() * state +
                   (s.mass / s.dt).asDiagonal() * state_velocity +
                   s.mass.asDiagonal() * state_acceleration;

  // Check if we have active barrier constraints
  int state_num = state.size();
  bool has_active_barriers = false;
  for (int i = 0; i < state_num; ++i) {
    if (barrier_lhs(i) != 0.0f) {
      has_active_barriers = true;
      break;
    }
  }

  if (!has_active_barriers) {
    s.has_barrier_constrain = false;
    // Use base diagonal
    s.HB_diag = s.H_diag;
    return true;
  }

  // Update diagonal with barrier weights: HB = H + barrier_lhs
  s.has_barrier_constrain = true;
  s.HB_diag = s.H_diag + barrier_lhs;

  // Add barrier contribution to RHS
  rhs += barrier_rhs;

  return true;
}

bool batch_compute_gpu_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto obj_state = registry.get<ObjectState>(e);
    auto dynamic_data = registry.get<GpuClothSolverContext>(e);

    if (obj_state && dynamic_data) {
      auto seq = Eigen::seqN(obj_state->state_offset, obj_state->state_num);
      if (!compute_gpu_cloth_outer_loop(
              global_state(seq), global_state_velocity(seq),
              global_state_acceleration(seq), barrier_constrain.lhs(seq),
              barrier_constrain.rhs(seq), *dynamic_data, rhs(seq))) {
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief Inner loop for single GPU cloth - solve linear system using GPU Jacobi.
 *
 * Note: For simplicity, this version directly solves the linear system without
 * the full projective dynamics local/global iteration. The in-plane elastic
 * projection could be added as a future enhancement.
 */
bool compute_gpu_cloth_inner_loop(const ClothConfig& config,
                                  const RMatrixX3i& F,
                                  const ClothTopology& topology,
                                  const GpuClothSolverContext& solver_context,
                                  Eigen::Ref<const Eigen::VectorXf> state,
                                  Eigen::Ref<const Eigen::VectorXf> outer_rhs,
                                  Eigen::Ref<Eigen::VectorXf> solution) {
  auto& s = solver_context;

  // For now, we skip the projective dynamics local step (SVD projection)
  // and directly solve the linear system Hx = rhs using GPU Jacobi iteration.
  // This is a simplification - full projective dynamics would project elastic
  // constraints per-triangle before solving.

  Eigen::VectorXf rhs = outer_rhs;

  // GPU Jacobi solve: (D + R) * x = rhs
  int max_iter = 100;
  float tol = 1e-6f;

  GpuClothSolverContext& mutable_context = const_cast<GpuClothSolverContext&>(s);

  // Choose diagonal based on whether barriers are active and call solve
  // Extract diagonal reference to avoid Eigen::Ref type issues with NVCC
  const Eigen::VectorXf& diag = mutable_context.has_barrier_constrain
                                 ? mutable_context.HB_diag
                                 : mutable_context.H_diag;

  bool solve_success = mutable_context.jacobi_solver.solve(diag, rhs, solution, max_iter, tol);

  if (!solve_success) {
    SPDLOG_ERROR("GPU Jacobi solve failed");
    return false;
  }

  return true;
}

bool batch_compute_gpu_cloth_inner_loop(Registry& registry,
                                        const Eigen::VectorXf& global_state,
                                        const Eigen::VectorXf& outer_rhs,
                                        Eigen::VectorXf& solution) {
  for (Entity& e : registry.get_all_entities()) {
    auto obj_state = registry.get<ObjectState>(e);
    auto cloth_config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto topology = registry.get<ClothTopology>(e);
    auto dynamic_data = registry.get<GpuClothSolverContext>(e);

    if (obj_state && cloth_config && mesh && topology && dynamic_data) {
      auto seq = Eigen::seqN(obj_state->state_offset, obj_state->state_num);
      if (!compute_gpu_cloth_inner_loop(*cloth_config, mesh->F, *topology,
                                        *dynamic_data, global_state(seq),
                                        outer_rhs(seq), solution(seq))) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace silk
