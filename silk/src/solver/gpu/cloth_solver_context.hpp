/** @file
 * GPU cloth solver context - dynamic, per-cloth solver state.
 *
 * This is the GPU equivalent of CpuClothSolverContext, using GPU Jacobi
 * iteration instead of CPU CHOLMOD Cholesky factorization.
 */

#pragma once

#include <Eigen/Core>
#include <optional>

#include "cloth_topology.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver/gpu/jacobi_solver.cuh"

namespace silk {

/**
 * @brief Dynamic, time-step-dependent solver state for GPU cloth simulation.
 *
 * Stores solver data that changes with time step or configuration:
 * - Mass vector
 * - System matrix H (stored implicitly in GPU Jacobi solver)
 * - GPU Jacobi solver state
 * - Weighted rest-curvature vector
 *
 * Notation: state_num = 3 * vertex_num
 */
class GpuClothSolverContext {
 public:
  // Time step in seconds
  float dt;

  // Whether barrier constraints are currently active
  bool has_barrier_constrain;

  // True vectorized per-vertex mass vector (density-scaled) of length state_num
  Eigen::VectorXf mass;

  // Left-hand side H of Hx = b, combining momentum, bending, in-plane, and pin
  // energies. Stored in CPU memory for building, then passed to GPU solver.
  Eigen::SparseMatrix<float> H;

  // GPU Jacobi solver managing device memory and iteration state
  gpu::GpuJacobiSolver jacobi_solver;

  // Diagonal extracted from H (for Jacobi iteration)
  Eigen::VectorXf H_diag;

  // Updated diagonal to account for barrier constraints
  Eigen::VectorXf HB_diag;

  // Weighted rest-curvature vector (state_num x 1)
  Eigen::VectorXf C0;

 public:
  /**
   * @brief Build dynamic, time-step-dependent solver context for cloth.
   *
   * Assembles the system matrix H, extracts its diagonal, and initializes
   * the GPU Jacobi solver with the matrix structure.
   *
   * @param config Cloth physical properties
   * @param topology Precomputed mesh topology data
   * @param pin Pinned vertex constraints
   * @param dt Time step in seconds
   * @return std::nullopt if setup fails, otherwise initialized context
   */
  static std::optional<GpuClothSolverContext> make_cloth_solver_context(
      const ClothConfig& config, const ClothTopology& topology, const Pin& pin,
      float dt);

 private:
  GpuClothSolverContext() = default;
};

}  // namespace silk
