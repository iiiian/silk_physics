#include "cloth_solver.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <omp.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <optional>
#include <unsupported/Eigen/KroneckerProduct>

#include "../barrier_constrain.hpp"
#include "../cholmod_utils.hpp"
#include "../cloth_solver_data.hpp"
#include "../eigen_utils.hpp"
#include "../logger.hpp"
#include "../mesh.hpp"
#include "../pin.hpp"
#include "../solver_state.hpp"
#include "silk/silk.hpp"

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
    SPDLOG_DEBUG("degenreate triangle in cloth mesh");
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

/** Build static, geometry‑dependent solver data for a cloth.
 *  - `mass` uses Voronoi vertex masses scaled by density.
 *  - `CWC` is the mass‑weighted cotangent Laplacian used for bending.
 *  - `JWJ` assembles the in‑plane elastic quadratic form from per‑face
 *    Jacobians and triangle areas.
 */
ClothStaticSolverData make_cloth_static_solver_data(const ClothConfig& config,
                                                    const TriMesh& mesh) {
  const ClothConfig& c = config;
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

  ClothStaticSolverData d;
  d.mass = c.density * voroni_mass.diagonal();
  d.area = std::move(area);
  d.CWC = std::move(CWC);
  d.JWJ = std::move(JWJ);
  d.jacobian_ops = std::move(jops);
  d.C0 = std::move(C0);

  return d;
}

/** Build dynamic, time‑step‑dependent or config-dependent solver data for a
 * cloth.
 *
 *  Returns std::nullopt if analysis/factorization fails.
 */
std::optional<ClothDynamicSolverData> make_cloth_dynamic_solver_data(
    const ClothConfig& config, const ClothStaticSolverData& static_data,
    const Pin& pin, float dt) {
  auto& c = config;
  auto& s = static_data;

  int state_num = 3 * s.mass.size();
  std::vector<Eigen::Triplet<float>> H_triplets;

  // Assemble momentum term.
  Eigen::VectorXf M = 1.0f / (dt * dt) * s.mass;
  for (int i = 0; i < M.size(); ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }
  // Assemble in-plane elastic energy term.
  append_triplets_from_sparse(s.JWJ, 0, 0, c.elastic_stiffness, H_triplets,
                              Symmetry::Upper);
  // Assemble bending energy term.
  append_triplets_from_vectorized_sparse(s.CWC, 0, 0, c.bending_stiffness,
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

  ClothDynamicSolverData d;
  d.dt = dt;
  d.has_barrier_constrain = false;
  d.H = std::move(H);
  d.L = std::move(L);
  d.LB = {};
  d.C0 = (c.bending_stiffness * s.C0).reshaped<Eigen::RowMajor>();

  return d;
}

void reset_all_cloth_for_solver(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto mesh = registry.get<TriMesh>(e);
    auto solver_state = registry.get<SolverState>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);

    if (mesh && solver_state && dynamic_data) {
      solver_state->state_offset = 0;
      solver_state->curr_state = mesh->V.reshaped<Eigen::RowMajor>();
      solver_state->state_velocity.setZero();

      // Clear any barrier updates on the cached factorization.
      dynamic_data->has_barrier_constrain = false;
      dynamic_data->LB = {};
    }
  }
}

bool init_all_cloth_for_solver(Registry& registry, float dt) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto pin = registry.get<Pin>(e);
    if (!(config && mesh && pin)) {
      continue;
    }

    auto solver_state = registry.get<SolverState>(e);
    if (!solver_state) {
      SolverState new_solver_state;
      new_solver_state.state_offset = 0;
      new_solver_state.state_num = 3 * mesh->V.rows();
      new_solver_state.curr_state = mesh->V.reshaped<Eigen::RowMajor>();
      new_solver_state.state_velocity =
          Eigen::VectorXf::Zero(new_solver_state.state_num);
      solver_state = registry.set(e, std::move(new_solver_state));
    }

    auto static_data = registry.get<ClothStaticSolverData>(e);
    if (!static_data) {
      static_data =
          registry.set(e, make_cloth_static_solver_data(*config, *mesh));
    }
    assert((static_data != nullptr));

    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);
    if (dynamic_data) {
      dynamic_data->has_barrier_constrain = false;
      dynamic_data->LB = {};
    } else {
      auto new_dynamic_data =
          make_cloth_dynamic_solver_data(*config, *static_data, *pin, dt);
      // Fail fast if factorization did not succeed (non‑SPD or ill‑posed).
      if (!new_dynamic_data) {
        return false;
      }
      registry.set<ClothDynamicSolverData>(e, std::move(*new_dynamic_data));
    }
  }

  return true;
}

void compute_cloth_init_rhs(const ClothDynamicSolverData dynamic_data,
                            const Pin& pin,
                            Eigen::Ref<Eigen::VectorXf> init_rhs) {
  init_rhs.setZero();

  // set pin rhs
  for (int i = 0; i < pin.index.size(); ++i) {
    init_rhs(Eigen::seqN(3 * pin.index(i), 3)) =
        pin.pin_stiffness * pin.position(Eigen::seqN(3 * i, 3));
  }

  // set rest curvature rhs
  init_rhs += dynamic_data.C0;
}

void compute_all_cloth_init_rhs(Registry& registry, Eigen::VectorXf& init_rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto solver_state = registry.get<SolverState>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);
    auto pin = registry.get<Pin>(e);

    if (solver_state && dynamic_data && pin) {
      auto seq =
          Eigen::seqN(solver_state->state_offset, solver_state->state_num);
      compute_cloth_init_rhs(*dynamic_data, *pin, init_rhs(seq));
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
    const ClothStaticSolverData& static_data,
    ClothDynamicSolverData& dynamic_data, Eigen::Ref<Eigen::VectorXf> rhs) {
  auto& s = static_data;
  auto& d = dynamic_data;

  // Momentum contribution to RHS (implicit Euler style aggregation).
  auto vectorized_mass = s.mass.replicate(1, 3).reshaped<Eigen::RowMajor>();
  rhs.noalias() += (vectorized_mass / (d.dt * d.dt)).asDiagonal() * state +
                   (vectorized_mass / d.dt).asDiagonal() * state_velocity +
                   vectorized_mass.asDiagonal() * state_acceleration;

  int state_num = state.size();

  // Barrier constraint update (diagonal; skip inactive entries).
  Eigen::SparseMatrix<float> C{state_num, state_num};
  for (int i = 0; i < state_num; ++i) {
    // Inactive entry.
    if (barrier_lhs(i) == 0) {
      continue;
    }

    C.coeffRef(i, i) = barrier_lhs(i);
  }

  if (C.nonZeros() == 0) {
    d.has_barrier_constrain = false;
    return true;
  }
  d.has_barrier_constrain = true;

  // Barrier contribution to rhs.
  rhs += barrier_rhs;

  // Update Cholesky factorization using CHOLMOD up/down.
  C = C.cwiseSqrt();
  cholmod_sparse C_view = cholmod_raii::make_cholmod_sparse_view(C);

  // Permute C according to L.Perm as suggested by CHOLMOD for up/down updates.
  int32_t* rset = static_cast<int32_t*>(d.L.raw()->Perm);
  int64_t rset_num = static_cast<int64_t>(d.L.raw()->n);
  cholmod_raii::CholmodSparse C_perm = cholmod_submatrix(
      &C_view, rset, rset_num, nullptr, -1, 1, 1, cholmod_raii::common);
  if (C_perm.is_empty()) {
    return false;
  }

  d.LB = d.L;
  if (!cholmod_updown(1, C_perm, d.LB, cholmod_raii::common)) {
    return false;
  }

  return true;
}

bool compute_all_cloth_outer_loop(
    Registry& registry, const Eigen::VectorXf& global_state,
    const Eigen::VectorXf& global_state_velocity,
    const Eigen::VectorXf& global_state_acceleration,
    const BarrierConstrain& barrier_constrain, Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto solver_state = registry.get<SolverState>(e);
    auto static_data = registry.get<ClothStaticSolverData>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);

    if (solver_state && static_data && dynamic_data) {
      auto seq =
          Eigen::seqN(solver_state->state_offset, solver_state->state_num);
      if (!compute_cloth_outer_loop(
              global_state(seq), global_state_velocity(seq),
              global_state_acceleration(seq), barrier_constrain.lhs(seq),
              barrier_constrain.rhs(seq), *static_data, *dynamic_data,
              rhs(seq))) {
        return false;
      }
    }
  }

  return true;
}

/** Inner loop: local projective step  followed by a global linear solve using
 *  the cached factorization.
 */
bool compute_cloth_inner_loop(const ClothConfig& config, const RMatrixX3i& F,
                              const ClothStaticSolverData& static_data,
                              const ClothDynamicSolverData& dynamic_data,
                              Eigen::Ref<const Eigen::VectorXf> state,
                              Eigen::Ref<const Eigen::VectorXf> outer_rhs,
                              Eigen::Ref<Eigen::VectorXf> solution) {
  auto& s = static_data;
  auto& d = dynamic_data;

  int state_num = state.size();

  // Project in‑plane elastic constraint (per‑face, threaded accumulation).
  std::vector<Eigen::VectorXf> thread_local_rhs(
      omp_get_max_threads(), Eigen::VectorXf::Zero(state_num));
#pragma omp parallel for
  for (int i = 0; i < s.jacobian_ops.size(); ++i) {
    Eigen::Vector3i offset = 3 * F.row(i);

    // Assemble local vectorized vertex position.
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = state(Eigen::seqN(offset(0), 3));
    buffer(Eigen::seqN(3, 3)) = state(Eigen::seqN(offset(1), 3));
    buffer(Eigen::seqN(6, 3)) = state(Eigen::seqN(offset(2), 3));

    // Deformation matrix.
    Eigen::Matrix<float, 3, 2> D = (s.jacobian_ops[i] * buffer).reshaped(3, 2);
    // SVD of deformation. Clamp singular values to form the projected target T
    // (rotation + limited stretch). Eigen does not return thin U/V for 3x2, so
    // compute full and slice.
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
    float weight = config.elastic_stiffness * s.area(i);
    buffer = weight * s.jacobian_ops[i].transpose() * T.reshaped();

    auto& local_rhs = thread_local_rhs[omp_get_thread_num()];
    local_rhs(Eigen::seqN(offset(0), 3)) += buffer(Eigen::seqN(0, 3));
    local_rhs(Eigen::seqN(offset(1), 3)) += buffer(Eigen::seqN(3, 3));
    local_rhs(Eigen::seqN(offset(2), 3)) += buffer(Eigen::seqN(6, 3));
  }

  Eigen::VectorXf rhs = outer_rhs;
  for (auto& local_rhs : thread_local_rhs) {
    rhs += local_rhs;
  }

  // Global solve with (optionally barrier‑updated) Cholesky factorization.
  cholmod_dense rhs_view = cholmod_raii::make_cholmod_dense_vector_view(rhs);
  auto& L = (d.has_barrier_constrain) ? d.LB : d.L;

  cholmod_raii::CholmodDense cholmod_solution =
      cholmod_solve(CHOLMOD_A, L, &rhs_view, cholmod_raii::common);
  if (cholmod_solution.is_empty()) {
    return false;
  }

  solution = cholmod_raii::make_eigen_dense_vector_view(cholmod_solution);

  return true;
}

bool compute_all_cloth_inner_loop(Registry& registry,
                                  const Eigen::VectorXf& global_state,
                                  const Eigen::VectorXf& outer_rhs,
                                  Eigen::VectorXf& out) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto static_data = registry.get<ClothStaticSolverData>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);
    auto solver_state = registry.get<SolverState>(e);

    if (!(config && mesh && static_data && dynamic_data && solver_state)) {
      continue;
    }

    auto seq = Eigen::seqN(solver_state->state_offset, solver_state->state_num);
    if (!compute_cloth_inner_loop(*config, mesh->F, *static_data, *dynamic_data,
                                  global_state(seq), outer_rhs(seq),
                                  out(seq))) {
      return false;
    }
  }

  return true;
}
}  // namespace silk
