#include "solver/cpu/cloth_solver_context.hpp"

#include <Eigen/Core>
#include <optional>

#include "cloth_topology.hpp"
#include "eigen_utils.hpp"
#include "logger.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver/cpu/cholmod_utils.hpp"

namespace silk {

std::optional<CpuClothSolverContext>
CpuClothSolverContext::make_cloth_solver_context(const ClothConfig& config,
                                                 const ClothTopology& topology,
                                                 const Pin& pin, float dt) {
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

  CpuClothSolverContext ctx;
  ctx.dt = dt;
  ctx.has_barrier_constrain = false;
  ctx.mass = std::move(vec_mass);
  ctx.H = std::move(H);
  ctx.L = std::move(L);
  ctx.LB = {};
  ctx.C0 = (c.bending_stiffness * t.C0).reshaped<Eigen::RowMajor>();

  return ctx;
}

}  // namespace silk
