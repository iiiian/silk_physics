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
#include "silk/silk.hpp"

namespace silk {

std::optional<Eigen::Matrix<float, 6, 9>> triangle_jacobian_operator(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2, float zero_threshold) {
  // convert triangle to 2D
  // use edge e1 = v1 -> v2 as x axis
  // use n x e1 as y axis
  Eigen::Vector3f e0 = v1 - v0;
  Eigen::Vector3f e1 = v2 - v0;

  // basis x
  Eigen::Vector3f bx = e0.normalized();
  // basis y
  Eigen::Vector3f e0xe1 = e0.cross(e1);
  if (e0xe1.norm() < zero_threshold) {
    return std::nullopt;
  }
  Eigen::Vector3f by = e0xe1.cross(e0).normalized();

  // dX is the displacement of the initial triangle in 2D basis
  // dX = ( d1 d2 )
  Eigen::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e0);
  dX(1, 0) = 0.0f;
  dX(0, 1) = bx.dot(e1);
  dX(1, 1) = by.dot(e1);

  // clang-format off
  // D is the displace operator in 3D cartesian basis
  // dx = ( d1 d2 ) = x * D where x = ( v1 v2 v3 )
  const Eigen::Matrix<float, 3, 2> D =
        (Eigen::Matrix<float, 3, 2>() << -1, -1,
                                          1,  0,
                                          0,  1).finished();
  // clang-format on

  // deformation F = dx * (dX)^-1 = x * D * (dX)^-1
  // here we ignore the difference in basis between dx and dX since the
  // additional transformation will be canceled out in later stage of algorithm.
  // use kronecker product to vectorize above equation:
  // given (B^T ⊗  A) vec(X) = vec(AXB),
  // F = x * D * (dX)^-1 = ((D * (dX)^-1)^T ⊗ I3 * vec(x).
  // so the jacobian operator J = ((D * (dX)^-1)^T ⊗ I3
  Eigen::Matrix<float, 2, 3> B = (D * dX.inverse()).transpose();
  return Eigen::KroneckerProduct(B, Eigen::Matrix3f::Identity());
}

ClothStaticSolverData make_cloth_static_solver_data(const ClothConfig& config,
                                                    const TriMesh& mesh) {
  const ClothConfig& c = config;
  const TriMesh& m = mesh;

  int vnum = m.V.rows();
  int fnum = m.F.rows();

  Eigen::SparseMatrix<float> voroni_mass;
  igl::massmatrix(m.V, m.F, igl::MASSMATRIX_TYPE_VORONOI, voroni_mass);

  // cotangent matrix
  Eigen::SparseMatrix<float> C;
  igl::cotmatrix(m.V, m.F, C);

  // cotangent laplacian weight
  Eigen::SparseMatrix<float> W(vnum, vnum);
  W.setIdentity();
  for (int i = 0; i < vnum; ++i) {
    W.coeffRef(i, i) = 1.0f / voroni_mass.coeff(i, i);
  }

  // this is the weighted AA for bending energy.
  // assume initial curvature is 0 so there is no solver constrain for bending
  Eigen::SparseMatrix<float> CWC = C.transpose() * W * C;
  RMatrixX3f C0 = CWC * m.V;

  // triangle area
  Eigen::VectorXf area;
  igl::doublearea(m.V, m.F, area);
  area /= 2;

  // in-plane deformation energy
  std::vector<Eigen::Triplet<float>> JWJ_triplets;
  std::vector<Eigen::Matrix<float, 6, 9>> jops;
  for (int f = 0; f < fnum; ++f) {
    // TODO: avoid hardcoded jacobian zero_threshold
    auto jop = triangle_jacobian_operator(
        m.V.row(m.F(f, 0)), m.V.row(m.F(f, 1)), m.V.row(m.F(f, 2)), 0.0f);
    if (!jop) {
      // TODO: handle degenerate triangle better?
      continue;
    }

    float weight = area(f);
    Eigen::Matrix<float, 9, 9> local_AA = weight * (*jop).transpose() * (*jop);
    jops.push_back(*jop);

    // convert local AA to global AA
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            float val = local_AA(3 * vi + i, 3 * vj + j);
            // TODO: avoid hardcoded zero threshold
            if (std::abs(val) == 0.0f) {
              continue;
            }
            // TODO: consider purging value near zero to save compute
            JWJ_triplets.emplace_back(3 * m.F(f, vi) + i, 3 * m.F(f, vj) + j,
                                      val);
          }
        }
      }
    }
  }

  Eigen::SparseMatrix<float> JWJ{3 * vnum, 3 * vnum};
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

std::optional<ClothDynamicSolverData> make_cloth_dynamic_solver_data(
    const ClothConfig& config, const TriMesh& mesh,
    const ClothStaticSolverData& static_data, const Pin& pin, float dt) {
  auto& c = config;
  auto& s = static_data;

  int state_num = 3 * mesh.V.rows();
  std::vector<Eigen::Triplet<float>> H_triplets;

  // assemble momentum term
  Eigen::VectorXf M = 1 / (dt * dt) * s.mass;
  for (int i = 0; i < state_num; ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }
  // assemble in-plane elastic energy term
  append_triplets_from_sparse(static_data.JWJ, 0, 0, 1.0f, H_triplets,
                              Symmetry::Upper);
  // assemble bending energy term
  append_triplets_from_vectorized_sparse(
      static_data.CWC, 0, 0, c.bending_stiffness, H_triplets, Symmetry::Upper);
  // assemble pin term
  for (int i = 0; i > pin.index.size(); ++i) {
  }

  Eigen::SparseMatrix<float> H{state_num, state_num};
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  auto H_view = cholmod_raii::make_cholmod_sparse_view(H, Symmetry::Upper);
  cholmod_raii::CholmodFactor L =
      cholmod_analyze(&H_view, cholmod_raii::common);
  if (L.is_empty()) {
    return std::nullopt;
  }
  int r = cholmod_factorize(&H_view, L, cholmod_raii::common);
  if (r != 0) {
    return std::nullopt;
  }

  ClothDynamicSolverData d;
  d.state_num = state_num;
  d.state_offset = 0;
  d.dt = dt;
  d.has_barrier_constrain = false;
  d.L = std::move(L);
  d.LB = {};
  d.init_state = mesh.V.reshaped<Eigen::RowMajor>();
  d.curr_state = d.init_state;
  d.state_velocity = Eigen::VectorXf::Zero(state_num);
  return d;
}

void reset_cloth_dynamic_solver_data(ClothDynamicSolverData& data) {
  data.state_offset = 0;
  data.has_barrier_constrain = false;
  data.LB = {};
  data.curr_state = data.init_state;
  data.state_velocity = Eigen::VectorXf::Zero(data.state_num);
}

bool init_all_cloth_solver_data(Registry& registry, float dt) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto pin = registry.get<Pin>(e);
    if (!(config && mesh && pin)) {
      continue;
    }

    auto static_data = registry.get<ClothStaticSolverData>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);

    if (!static_data) {
      registry.set(e, make_cloth_static_solver_data(*config, *mesh));
      static_data = registry.get<ClothStaticSolverData>(e);
    }

    assert((static_data != nullptr));
    if (dynamic_data) {
      reset_cloth_dynamic_solver_data(*dynamic_data);
    } else {
      auto new_dynamic_data = make_cloth_dynamic_solver_data(
          *config, *mesh, *static_data, *pin, dt);
      // fails when cholmod fail to factorize
      if (!new_dynamic_data) {
        return false;
      }
      registry.set<ClothDynamicSolverData>(e, std::move(*new_dynamic_data));
    }
  }

  return true;
}

bool init_cloth_outer_loop(const Eigen::VectorXf& solver_state,
                           const Eigen::VectorXf& solver_state_velocity,
                           const Eigen::VectorXf& solver_state_acceleration,
                           const BarrierConstrain& barrier_constrain,
                           const ClothStaticSolverData& static_data,
                           ClothDynamicSolverData& dynamic_data,
                           Eigen::VectorXf& rhs) {
  auto& s = static_data;
  auto& d = dynamic_data;
  auto& b = barrier_constrain;

  auto seq = Eigen::seqN(d.state_offset, d.state_num);

  // compute mometum component of rhs
  auto position =
      solver_state(seq).reshaped<Eigen::RowMajor>(d.state_num / 3, 3);
  auto velocity =
      solver_state_velocity(seq).reshaped<Eigen::RowMajor>(d.state_num / 3, 3);
  auto acceleration = solver_state_acceleration(seq).reshaped<Eigen::RowMajor>(
      d.state_num / 3, 3);

  rhs += (s.mass / (d.dt * d.dt)).asDiagonal() * position +
         (s.mass / d.dt).asDiagonal() * velocity +
         s.mass.asDiagonal() * acceleration;

  // compute barrier constrain update
  Eigen::SparseMatrix<float> C;
  for (int i = 0; i < d.state_num; ++i) {
    if (b.lhs(d.state_offset + i) == 0) {
      continue;
    }

    C.coeffRef(i, i) = b.lhs(d.state_offset + i);
  }

  if (C.nonZeros() == 0) {
    d.has_barrier_constrain = false;
    return true;
  }
  d.has_barrier_constrain = true;

  // barrier lhs update
  C = C.cwiseSqrt();
  cholmod_sparse C_view = cholmod_raii::make_cholmod_sparse_view(C);
  int32_t* rset = static_cast<int32_t*>(d.L.raw()->Perm);
  int64_t rset_num = static_cast<int64_t>(d.L.raw()->n);
  cholmod_raii::CholmodSparse C_perm = cholmod_submatrix(
      &C_view, rset, rset_num, nullptr, -1, 1, 1, cholmod_raii::common);
  d.LB = d.L;
  if (C_perm.is_empty()) {
    return false;
  }

  int result = cholmod_updown(1, C_perm, d.LB, cholmod_raii::common);
  if (result == 0) {
    return false;
  }

  // barrier rhs update
  rhs += b.rhs;

  return true;
}

bool init_all_cloth_outer_loop(Registry& registry,
                               const Eigen::VectorXf& solver_state,
                               const Eigen::VectorXf& solver_state_velocity,
                               const Eigen::VectorXf& solver_state_acceleration,
                               const BarrierConstrain& barrier_constrain,
                               Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto static_data = registry.get<ClothStaticSolverData>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);

    if (static_data && dynamic_data) {
      bool success = init_cloth_outer_loop(
          solver_state, solver_state_velocity, solver_state_acceleration,
          barrier_constrain, *static_data, *dynamic_data, rhs);
      if (success) {
        return false;
      }
    }
  }

  return true;
}

bool solve_cloth_inner_loop(const ClothConfig& config, const TriMesh& mesh,
                            const ClothStaticSolverData& static_data,
                            const ClothDynamicSolverData& dynamic_data,
                            const Eigen::VectorXf& solver_state,
                            const Eigen::VectorXf& init_rhs,
                            Eigen::VectorXf& out) {
  auto& s = static_data;
  auto& d = dynamic_data;
  auto& F = mesh.F;

  // project in-plane elastic constrain
  std::vector<Eigen::VectorXf> thread_local_rhs(
      omp_get_max_threads(), Eigen::VectorXf::Zero(d.state_num));
#pragma omp parallel for
  for (int i = 0; i < s.jacobian_ops.size(); ++i) {
    Eigen::Vector3i offset = d.state_offset + 3 * F.row(i).array();

    // assemble local vectorized vertex position
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = solver_state(Eigen::seqN(offset(0), 3));
    buffer(Eigen::seqN(3, 3)) = solver_state(Eigen::seqN(offset(1), 3));
    buffer(Eigen::seqN(6, 3)) = solver_state(Eigen::seqN(offset(2), 3));

    // deformation matrix
    Eigen::Matrix<float, 3, 2> F = (s.jacobian_ops[i] * buffer).reshaped(3, 2);
    // SVD decompose deformation.
    // replacing diagonal term with idenity gives us the projection
    // eigen can't compute thin U and V. so instead compute full UV
    Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
        F, Eigen::ComputeFullV | Eigen::ComputeFullU);
    // this is the projection, deformation F cause by purely rotation +
    // translation
    Eigen::Vector2f sigma = svd.singularValues();
    sigma(0) = std::clamp(sigma(0), 0.9f, 1.1f);
    sigma(1) = std::clamp(sigma(1), 0.9f, 1.1f);
    Eigen::Matrix<float, 3, 2> T = svd.matrixU().block<3, 2>(0, 0) *
                                   sigma.asDiagonal() *
                                   svd.matrixV().transpose();

    // compute the elastic rhs, reuse buffer
    float weight = config.elastic_stiffness * s.area(i);
    buffer = weight * s.jacobian_ops[i].transpose() * T.reshaped();
    auto& local_rhs = thread_local_rhs[omp_get_thread_num()];
    // add it back to thread local rhs
    local_rhs(Eigen::seqN(offset(0), 3)) += buffer(Eigen::seqN(0, 3));
    local_rhs(Eigen::seqN(offset(1), 3)) += buffer(Eigen::seqN(3, 3));
    local_rhs(Eigen::seqN(offset(2), 3)) += buffer(Eigen::seqN(6, 3));
  }

  auto seq = Eigen::seqN(d.state_offset, d.state_num);
  Eigen::VectorXf rhs = init_rhs(seq);
  for (auto& local_rhs : thread_local_rhs) {
    rhs += local_rhs;
  }

  // global solve
  cholmod_dense rhs_view = cholmod_raii::make_cholmod_dense_vector_view(rhs);
  auto& L = (d.has_barrier_constrain) ? d.LB : d.L;

  cholmod_raii::CholmodDense sol =
      cholmod_solve(CHOLMOD_A, L, &rhs_view, cholmod_raii::common);
  if (sol.is_empty()) {
    return false;
  }

  out(seq) = cholmod_raii::make_eigen_dense_vector_view(sol);

  return true;
}

bool solve_all_cloth_inner_loop(Registry& registry,
                                const Eigen::VectorXf& solver_state,
                                const Eigen::VectorXf& init_rhs,
                                Eigen::VectorXf& out) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto static_data = registry.get<ClothStaticSolverData>(e);
    auto dynamic_data = registry.get<ClothDynamicSolverData>(e);

    if (!(config && mesh && static_data && dynamic_data)) {
      continue;
    }

    bool success =
        solve_cloth_inner_loop(*config, *mesh, *static_data, *dynamic_data,
                               solver_state, init_rhs, out);
    if (!success) {
      return false;
    }
  }

  return true;
}
}  // namespace silk
