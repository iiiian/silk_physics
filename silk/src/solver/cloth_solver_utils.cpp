#include "cloth_solver_utils.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <optional>
#include <silk/silk.hpp>
#include <unsupported/Eigen/KroneckerProduct>

#include "../barrier_constrain.hpp"
#include "../cholmod_utils.hpp"
#include "../cloth_solver_data.hpp"
#include "../eigen_utils.hpp"
#include "../logger.hpp"
#include "../mesh.hpp"
#include "../object_collider.hpp"
#include "../object_collider_utils.hpp"
#include "../object_state.hpp"
#include "../pin.hpp"

namespace silk {

/** Compute the Jacobian operator that maps stacked vertex positions of a
 *  triangle (9x1) to the 3x2 deformation gradient in a local 2D chart, and
 *  return it in vectorized form as a 6x9 matrix J such that vec(F) = J * x.
 *
 *  Preconditions: the triangle is non‑degenerate (non‑zero edge length and
 *  area). Basis differences between dx and dX are ignored intentionally since
 *  they cancel out in the quadratic form used later.
 */
Eigen::Matrix<float, 6, 9> triangle_jacobian_operator(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2) {
  // Convert triangle to 2D.
  // Use edge e1 = v1 -> v2 as x axis.
  // Use n x e1 as y axis.
  Eigen::Vector3f e0 = v1 - v0;
  Eigen::Vector3f e1 = v2 - v0;

  Eigen::Vector3f e0xe1 = e0.cross(e1);
  // We assume no degenerate triangle exists.
  float area2_eps =
      std::pow(1e-6f * std::max(e0.squaredNorm(), e1.squaredNorm()), 2);
  if (e0xe1.squaredNorm() < area2_eps) {
    SPDLOG_DEBUG("degenerate triangle in cloth mesh");
  }
  // Define 2D basis x and y.
  Eigen::Vector3f bx = e0.normalized();
  Eigen::Vector3f by = e0xe1.cross(e0).normalized();

  // The matrix dX is the displacement of the initial triangle in a 2D basis.
  // Definition: dX = ( d1 d2 ).
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e0);
  dX(1, 0) = 0.0f;
  dX(0, 1) = bx.dot(e1);
  dX(1, 1) = by.dot(e1);

  // clang-format off
  // D is the displacement operator in 3D Cartesian basis.
  // The matrix dx equals ( d1 d2 ) = x * D where x = ( v1 v2 v3 ).
  const Eigen::Matrix<float, 3, 2> D =
        (Eigen::Matrix<float, 3, 2>() << -1, -1,
                                          1,  0,
                                          0,  1).finished();
  // clang-format on

  // Deformation: F = dx * (dX)^-1 = x * D * (dX)^-1.
  // We vectorize via the Kronecker identity (B^T ⊗ A) vec(X) = vec(AXB).
  // We have vec(F) = ((D * (dX)^-1)^T ⊗ I3) vec(x) := J vec(x).
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

/** Build static, geometry‑dependent data for a cloth.
 *  - `mass` uses Voronoi vertex masses.
 *  - `CWC` is the mass‑weighted cotangent Laplacian used for bending.
 *  - `JWJ` assembles the in‑plane elastic quadratic form from per‑face
 *    Jacobians and triangle areas.
 */
ClothTopology make_cloth_topology(const ClothConfig& config,
                                  const TriMesh& mesh) {
  const TriMesh& m = mesh;

  int vert_num = m.V.rows();
  int face_num = m.F.rows();

  Eigen::SparseMatrix<float> voroni_mass;
  igl::massmatrix(m.V, m.F, igl::MASSMATRIX_TYPE_VORONOI, voroni_mass);

  // Cotangent matrix.
  Eigen::SparseMatrix<float> C;
  igl::cotmatrix(m.V, m.F, C);

  // Cotangent Laplacian weight.
  Eigen::SparseMatrix<float> W(vert_num, vert_num);
  W.setIdentity();
  for (int i = 0; i < vert_num; ++i) {
    W.coeffRef(i, i) = 1.0f / voroni_mass.coeff(i, i);
  }

  // Bending energy.
  Eigen::SparseMatrix<float> CWC = C.transpose() * W * C;
  RMatrixX3f C0 = CWC * m.V;

  // Triangle area.
  Eigen::VectorXf area;
  igl::doublearea(m.V, m.F, area);
  area /= 2;

  // In‑plane deformation energy.
  std::vector<Eigen::Triplet<float>> JWJ_triplets;
  std::vector<Eigen::Matrix<float, 6, 9>> jops;
  for (int f = 0; f < face_num; ++f) {
    Eigen::Matrix<float, 6, 9> jop = triangle_jacobian_operator(
        m.V.row(m.F(f, 0)), m.V.row(m.F(f, 1)), m.V.row(m.F(f, 2)));

    float weight = area(f);
    Eigen::Matrix<float, 9, 9> local_AA = weight * jop.transpose() * jop;
    jops.push_back(jop);

    // Scatter local AA into the global (vectorized-vertex) system.
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            float val = local_AA(3 * vi + i, 3 * vj + j);
            JWJ_triplets.emplace_back(3 * m.F(f, vi) + i, 3 * m.F(f, vj) + j,
                                      val);
          }
        }
      }
    }
  }

  Eigen::SparseMatrix<float> JWJ{3 * vert_num, 3 * vert_num};
  JWJ.setFromTriplets(JWJ_triplets.begin(), JWJ_triplets.end());

  ClothTopology t;
  t.mass = voroni_mass.diagonal();
  t.area = std::move(area);
  t.CWC = std::move(CWC);
  t.JWJ = std::move(JWJ);
  t.jacobian_ops = std::move(jops);
  t.C0 = std::move(C0);

  return t;
}

/** Build dynamic, time‑step‑dependent or config-dependent data for a
 * cloth.
 *
 *  Returns std::nullopt if analysis/factorization fails.
 */
std::optional<ClothSolverContext> make_cloth_solver_context(
    const ClothConfig& config, const ClothTopology& topology, const Pin& pin,
    float dt) {
  auto& c = config;
  auto& t = topology;

  int state_num = 3 * t.mass.size();
  std::vector<Eigen::Triplet<float>> H_triplets;

  // Assemble momentum term.
  Eigen::VectorXf M = 1.0f / (dt * dt) * c.density * t.mass;
  for (int i = 0; i < M.size(); ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }
  // Assemble in-plane elastic energy term.
  append_triplets_from_sparse(t.JWJ, 0, 0, c.elastic_stiffness, H_triplets,
                              Symmetry::Upper);
  // Assemble bending energy term.
  append_triplets_from_vectorized_sparse(t.CWC, 0, 0, c.bending_stiffness,
                                         H_triplets, Symmetry::Upper);
  // Assemble pin term.
  for (int i = 0; i < pin.index.size(); ++i) {
    int offset = 3 * pin.index(i);
    H_triplets.emplace_back(offset, offset, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 1, offset + 1, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 2, offset + 2, pin.pin_stiffness);
  }

  Eigen::SparseMatrix<float> H{state_num, state_num};
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  // Compute Cholesky factorization (symbolic + numeric).
  auto H_view = cholmod_raii::make_cholmod_sparse_view(H, Symmetry::Upper);
  cholmod_raii::CholmodFactor L =
      cholmod_analyze(&H_view, cholmod_raii::common);
  if (L.is_empty()) {
    SPDLOG_DEBUG("cholmod analyze fail");
    return std::nullopt;
  }
  if (!cholmod_factorize(&H_view, L, cholmod_raii::common)) {
    SPDLOG_DEBUG("cholmod factorize fail");
    return std::nullopt;
  }

  Eigen::VectorXf vec_mass(state_num);
  for (int i = 0; i < t.mass.size(); ++i) {
    float val = c.density * t.mass(i);
    vec_mass(3 * i) = val;
    vec_mass(3 * i + 1) = val;
    vec_mass(3 * i + 2) = val;
  }

  ClothSolverContext ctx;
  ctx.dt = dt;
  ctx.has_barrier_constrain = false;
  ctx.mass = std::move(vec_mass);
  ctx.H = std::move(H);
  ctx.L = std::move(L);
  ctx.LB = {};
  ctx.C0 = (c.bending_stiffness * t.C0).reshaped<Eigen::RowMajor>();

  return ctx;
}

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
    topology = registry.set<ClothTopology>(
        e, make_cloth_topology(*cloth_config, *mesh));
  }
  assert(topology != nullptr);

  auto context = registry.get<ClothSolverContext>(e);
  if (context && context->dt == dt) {
    context->has_barrier_constrain = false;
  } else {
    auto new_context =
        make_cloth_solver_context(*cloth_config, *topology, *pin, dt);
    if (!new_context) {
      return false;
    }
    context = registry.set<ClothSolverContext>(e, std::move(*new_context));
  }
  assert(context != nullptr);

  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    auto new_collider = make_physical_object_collider(
        e.self, *collision_config, *mesh, *pin, context->mass, state_offset);
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

/** Outer loop: Accumulate global RHS terms and, if needed, update the
 *  factorization with diagonal barrier weights using CHOLMOD up/down.
 *
 *  Barrier handling: We build a diagonal C from `barrier_lhs` (skipping zeros)
 *  and then apply a CHOLMOD up/down update to the Cholesky factorization in
 *  the permuted order expected by CHOLMOD (using `L.Perm`). This realizes
 *  H' = H + C.
 */
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
    auto solver_state = registry.get<ObjectState>(e);
    auto dynamic_data = registry.get<ClothSolverContext>(e);

    if (solver_state && dynamic_data) {
      auto seq =
          Eigen::seqN(solver_state->state_offset, solver_state->state_num);
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

/** Inner loop: Project in-plane elastic constraints followed by a global linear
 * solve using the cached factorization.
 */
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
}  // namespace silk
