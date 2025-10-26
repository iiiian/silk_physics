/** @file
 * GPU cloth solver context implementation.
 */

#include "solver/gpu/cloth_solver_context.hpp"

#include <Eigen/Core>
#include <optional>

#include "cloth_topology.hpp"
#include "eigen_utils.hpp"
#include "logger.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"

namespace silk {

std::optional<GpuClothSolverContext>
GpuClothSolverContext::make_cloth_solver_context(const ClothConfig& config,
                                                 const ClothTopology& topology,
                                                 const Pin& pin, float dt) {
  auto& c = config;
  auto& t = topology;

  int state_num = 3 * t.mass.size();
  std::vector<Eigen::Triplet<float>> H_triplets;

  // Assemble momentum term: M / (dt^2)
  Eigen::VectorXf M = 1.0f / (dt * dt) * c.density * t.mass;
  for (int i = 0; i < M.size(); ++i) {
    H_triplets.emplace_back(3 * i, 3 * i, M(i));
    H_triplets.emplace_back(3 * i + 1, 3 * i + 1, M(i));
    H_triplets.emplace_back(3 * i + 2, 3 * i + 2, M(i));
  }

  // Assemble in-plane elastic energy term: k_elastic * J^T W J
  append_triplets_from_sparse(t.JWJ, 0, 0, c.elastic_stiffness, H_triplets,
                              Symmetry::Upper);

  // Assemble bending energy term: k_bending * C^T W C
  append_triplets_from_vectorized_sparse(t.CWC, 0, 0, c.bending_stiffness,
                                         H_triplets, Symmetry::Upper);

  // Assemble pin term: k_pin for pinned vertices
  for (int i = 0; i < pin.index.size(); ++i) {
    int offset = 3 * pin.index(i);
    H_triplets.emplace_back(offset, offset, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 1, offset + 1, pin.pin_stiffness);
    H_triplets.emplace_back(offset + 2, offset + 2, pin.pin_stiffness);
  }

  // Build H matrix (still sparse, but will be converted to row-major for GPU)
  Eigen::SparseMatrix<float> H{state_num, state_num};
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  // Convert to row-major for efficient GPU access
  Eigen::SparseMatrix<float, Eigen::RowMajor> H_row_major = H;

  // Create context
  GpuClothSolverContext ctx;
  ctx.dt = dt;
  ctx.has_barrier_constrain = false;

  // Vectorized mass (density-scaled, replicated for XYZ)
  Eigen::VectorXf vec_mass(state_num);
  for (int i = 0; i < t.mass.size(); ++i) {
    float val = c.density * t.mass(i);
    vec_mass(3 * i) = val;
    vec_mass(3 * i + 1) = val;
    vec_mass(3 * i + 2) = val;
  }
  ctx.mass = std::move(vec_mass);

  // Store H for later use
  ctx.H = std::move(H);

  // Initialize GPU Jacobi solver with H matrix structure
  if (!ctx.jacobi_solver.setup(H_row_major, ctx.H_diag)) {
    SPDLOG_ERROR("Failed to initialize GPU Jacobi solver");
    return std::nullopt;
  }

  // Initialize barrier diagonal as copy of base diagonal
  ctx.HB_diag = ctx.H_diag;

  // Weighted rest-curvature vector
  ctx.C0 = (c.bending_stiffness * t.C0).reshaped<Eigen::RowMajor>();

  return ctx;
}

}  // namespace silk
