#pragma once

#include <Eigen/Core>
#include <optional>

#include "backend/cpu/solver/cholmod_utils.hpp"
#include "common/cloth_topology.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cpu {

/**
 * Dynamic, time step or config dependent quantities used by the cloth solver.
 *
 * Notation:
 * state_num = 3 * vertex num.
 */
class ClothSolverContext {
 public:
  // Time step in seconds.
  float dt;
  // Whether barrier constraints are currently active.
  bool has_barrier_constrain;

  // True vectorized per-vertex mass vector (density-scaled) of length state
  // num.
  Eigen::VectorXf mass;

  // Left-hand side H of Hx = b, combining momentum, bending, in-plane, and pin
  // energies.
  Eigen::SparseMatrix<float> H;

  // Cholesky factorization of H produced via CHOLMOD.
  cholmod_raii::CholmodFactor L;

  // Updated factorization to account for barrier constraints.
  cholmod_raii::CholmodFactor LB;

  // Weighted rest-curvature vector (state_num x 1).
  Eigen::VectorXf C0;

 public:
  /** Build dynamic, time‑step‑dependent or config-dependent data for a
   * cloth.
   *
   *  Returns std::nullopt if analysis/factorization fails.
   */
  static std::optional<ClothSolverContext> make_cloth_solver_context(
      const ClothConfig& config, const ClothTopology& topology, const Pin& pin,
      float dt);

 private:
  ClothSolverContext() = default;
};

}  // namespace silk::cpu
