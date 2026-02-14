#include "backend/cpu/solver/cloth_solver_utils.hpp"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <cassert>
#include <optional>
#include <unsupported/Eigen/KroneckerProduct>

#include "backend/cpu/collision/object_collider.hpp"
#include "backend/cpu/object_state.hpp"
#include "backend/cpu/solver/barrier_constrain.hpp"
#include "backend/cpu/solver/cholmod_utils.hpp"
#include "backend/cpu/solver/cloth_solver_context.hpp"
#include "common/cloth_topology.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cpu {

void batch_reset_cloth_simulation(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto mesh = registry.get<TriMesh>(e);
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<ClothSolverContext>(e);

    if (mesh && state && context) {
      state->state_offset = 0;
      state->curr_state = mesh->V.reshaped<Eigen::RowMajor>();
      state->state_velocity.setZero();

      // Clear any barrier updates on the cached factorization.
      context->has_barrier_constrain = false;
      context->LB = {};
    }
  }
}

bool prepare_cloth_simulation(Registry& registry, Entity& entity, float dt,
                              int state_offset) {
  auto& e = entity;

  auto cloth_config = registry.get<ClothConfig>(e);
  auto collision_config = registry.get<CollisionConfig>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin = registry.get<Pin>(e);

  // Cloth entity sanity check.
  assert(cloth_config && collision_config && mesh && pin);

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

  auto topology = registry.get<ClothTopology>(e);
  if (!topology) {
    topology =
        registry.set<ClothTopology>(e, ClothTopology(*cloth_config, *mesh));
  }
  assert(topology != nullptr);

  auto context = registry.get<ClothSolverContext>(e);
  if (context && context->dt == dt) {
    context->has_barrier_constrain = false;
  } else {
    auto new_context = ClothSolverContext::make_cloth_solver_context(
        *cloth_config, *topology, *pin, dt);
    if (!new_context) {
      return false;
    }
    context = registry.set<ClothSolverContext>(e, std::move(*new_context));
  }
  assert(context != nullptr);

  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    auto new_collider = ObjectCollider(e.self, *collision_config, *mesh, *pin,
                                       context->mass, state_offset);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  } else {
    collider->state_offset = state_offset;
  }
  assert(collider != nullptr);

  return true;
}

void compute_cloth_invariant_rhs(const ClothSolverContext& solver_context,
                                 const Pin& pin,
                                 Eigen::Ref<Eigen::VectorXf> rhs) {
  rhs.setZero();

  // set pin rhs
  for (int i = 0; i < pin.index.size(); ++i) {
    rhs(Eigen::seqN(3 * pin.index(i), 3)) =
        pin.pin_stiffness * pin.position(Eigen::seqN(3 * i, 3));
  }

  // set rest curvature rhs
  rhs += solver_context.C0;
}

void batch_compute_cloth_invariant_rhs(Registry& registry,
                                       Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<ClothSolverContext>(e);
    auto pin = registry.get<Pin>(e);

    if (state && context && pin) {
      auto seq = Eigen::seqN(state->state_offset, state->state_num);
      compute_cloth_invariant_rhs(*context, *pin, rhs(seq));
    }
  }
}

/// Outer loop: Accumulate global RHS terms and, if needed, update the
///  factorization with diagonal barrier weights using CHOLMOD up/down.
///
///  Barrier handling: We build a diagonal C from `barrier_lhs` (skipping zeros)
///  and then apply a CHOLMOD up/down update to the Cholesky factorization in
///  the permuted order expected by CHOLMOD (using `L.Perm`). This realizes
///  H' = H + C.
bool compute_cloth_outer_loop(
    Eigen::Ref<const Eigen::VectorXf> state,
    Eigen::Ref<const Eigen::VectorXf> state_velocity,
    Eigen::Ref<const Eigen::VectorXf> state_acceleration,
    Eigen::Ref<const Eigen::VectorXf> barrier_lhs,
    Eigen::Ref<const Eigen::VectorXf> barrier_rhs,
    ClothSolverContext& solver_context, Eigen::Ref<Eigen::VectorXf> rhs) {
  auto& s = solver_context;

  // Momentum contribution to RHS.
  rhs.noalias() += (s.mass / (s.dt * s.dt)).asDiagonal() * state +
                   (s.mass / s.dt).asDiagonal() * state_velocity +
                   s.mass.asDiagonal() * state_acceleration;

  // Barrier constraint update (diagonal; skip inactive entries).
  int state_num = state.size();
  Eigen::SparseMatrix<float> C{state_num, state_num};
  for (int i = 0; i < state_num; ++i) {
    // Inactive entry.
    if (barrier_lhs(i) == 0) {
      continue;
    }

    C.coeffRef(i, i) = barrier_lhs(i);
  }

  if (C.nonZeros() == 0) {
    s.has_barrier_constrain = false;
    return true;
  }
  s.has_barrier_constrain = true;

  // Barrier contribution to rhs.
  rhs += barrier_rhs;

  // Update Cholesky factorization using CHOLMOD up/down.
  C = C.cwiseSqrt();
  cholmod_sparse C_view = cholmod_raii::make_cholmod_sparse_view(C);

  // Permute C according to L.Perm as suggested by CHOLMOD for up/down updates.
  int32_t* rset = static_cast<int32_t*>(s.L.raw()->Perm);
  int64_t rset_num = static_cast<int64_t>(s.L.raw()->n);
  cholmod_raii::CholmodSparse C_perm = cholmod_submatrix(
      &C_view, rset, rset_num, nullptr, -1, 1, 1, cholmod_raii::common);
  if (C_perm.is_empty()) {
    return false;
  }

  s.LB = s.L;
  if (!cholmod_updown(1, C_perm, s.LB, cholmod_raii::common)) {
    return false;
  }

  return true;
}

bool batch_compute_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto obj_state = registry.get<ObjectState>(e);
    auto dynamic_data = registry.get<ClothSolverContext>(e);

    if (obj_state && dynamic_data) {
      auto seq = Eigen::seqN(obj_state->state_offset, obj_state->state_num);
      if (!compute_cloth_outer_loop(
              global_state(seq), global_state_velocity(seq),
              global_state_acceleration(seq), barrier_constrain.lhs(seq),
              barrier_constrain.rhs(seq), *dynamic_data, rhs(seq))) {
        return false;
      }
    }
  }

  return true;
}

/// Inner loop: Project in-plane elastic constraints followed by a global linear
/// solve using the cached factorization.
bool compute_cloth_inner_loop(const ClothConfig& config, const RMatrixX3i& F,
                              const ClothTopology& topology,
                              const ClothSolverContext& solver_context,
                              Eigen::Ref<const Eigen::VectorXf> state,
                              Eigen::Ref<const Eigen::VectorXf> outer_rhs,
                              Eigen::Ref<Eigen::VectorXf> solution) {
  auto& t = topology;
  auto& s = solver_context;

  int state_num = state.size();

  // Project in‑plane elastic constraint (per‑face, threaded accumulation).

  tbb::enumerable_thread_specific<Eigen::VectorXf> thread_rhs(
      Eigen::VectorXf::Zero(state_num));
  int ops_num = t.jacobian_ops.size();
  tbb::parallel_for(0, ops_num, [&](int i) {
    Eigen::Vector3i offset = 3 * F.row(i);

    // Assemble local vectorized vertex position.
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = state(Eigen::seqN(offset(0), 3));
    buffer(Eigen::seqN(3, 3)) = state(Eigen::seqN(offset(1), 3));
    buffer(Eigen::seqN(6, 3)) = state(Eigen::seqN(offset(2), 3));

    // Deformation matrix.
    Eigen::Matrix<float, 3, 2> D = (t.jacobian_ops[i] * buffer).reshaped(3, 2);
    // SVD of deformation. Clamp singular values to form the projected
    // target T (rotation + limited stretch). Eigen does not return thin U/V
    // for 3x2, so compute full and slice.
    Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
        D, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Vector2f sigma = svd.singularValues();
    sigma(0) = std::clamp(sigma(0), 0.9f, 1.1f);
    sigma(1) = std::clamp(sigma(1), 0.9f, 1.1f);
    // This is the projection: deformation F caused by pure rotation and
    // translation.
    Eigen::Matrix<float, 3, 2> T = svd.matrixU().block<3, 2>(0, 0) *
                                   sigma.asDiagonal() *
                                   svd.matrixV().transpose();

    // Accumulate elastic RHS contribution, scatter back to global layout.
    float weight = config.elastic_stiffness * t.area(i);
    buffer = weight * t.jacobian_ops[i].transpose() * T.reshaped();

    auto& local_rhs = thread_rhs.local();
    local_rhs(Eigen::seqN(offset(0), 3)) += buffer(Eigen::seqN(0, 3));
    local_rhs(Eigen::seqN(offset(1), 3)) += buffer(Eigen::seqN(3, 3));
    local_rhs(Eigen::seqN(offset(2), 3)) += buffer(Eigen::seqN(6, 3));
  });

  Eigen::VectorXf rhs = outer_rhs;
  for (auto& local_rhs : thread_rhs) {
    rhs += local_rhs;
  }

  // Global solve with (optionally barrier‑updated) Cholesky factorization.
  cholmod_dense rhs_view = cholmod_raii::make_cholmod_dense_view(rhs);
  auto& L = (s.has_barrier_constrain) ? s.LB : s.L;

  cholmod_raii::CholmodDense cholmod_solution =
      cholmod_solve(CHOLMOD_A, L, &rhs_view, cholmod_raii::common);
  if (cholmod_solution.is_empty()) {
    return false;
  }

  solution = cholmod_raii::make_eigen_dense_vector_view(cholmod_solution);

  return true;
}

bool batch_compute_cloth_inner_loop(Registry& registry,
                                    const Eigen::VectorXf& global_state,
                                    const Eigen::VectorXf& outer_rhs,
                                    Eigen::VectorXf& solution) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto topology = registry.get<ClothTopology>(e);
    auto solver_context = registry.get<ClothSolverContext>(e);
    auto state = registry.get<ObjectState>(e);

    if (!(config && mesh && topology && solver_context && state)) {
      continue;
    }

    auto seq = Eigen::seqN(state->state_offset, state->state_num);
    if (!compute_cloth_inner_loop(*config, mesh->F, *topology, *solver_context,
                                  global_state(seq), outer_rhs(seq),
                                  solution(seq))) {
      return false;
    }
  }

  return true;
}
}  // namespace silk::cpu
