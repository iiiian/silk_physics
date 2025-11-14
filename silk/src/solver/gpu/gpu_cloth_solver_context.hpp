#pragma once

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <vector>

#include "cloth_topology.hpp"
#include "eigen_alias.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"

namespace silk {
namespace gpu {

/**
 * @brief GPU-specific data for cloth solver
 *
 * Manages device memory allocations for topology, state, and intermediate
 * buffers required for GPU-accelerated projective dynamics cloth simulation.
 *
 * This context maintains CUDA device pointers for all data needed by the
 * elastic RHS kernel, including:
 * - Face connectivity (F)
 * - Jacobian operators (per-face 6x9 matrices)
 * - Per-face areas
 * - Current state (vertex positions)
 * - RHS vectors
 *
 * Memory is automatically managed via RAII - allocated on construction and
 * freed on destruction.
 */
class GpuClothSolverContext {
 public:
  // --- Host-side metadata ---
  float dt;                 ///< Time step in seconds
  int state_num;            ///< Total DOFs (3 * vertex count)
  int ops_num;              ///< Number of triangle faces
  float elastic_stiffness;  ///< Material stiffness parameter

  // --- Device memory pointers ---
  int* d_F;                  ///< Face indices [ops_num * 3]
  float* d_state;            ///< Current vertex positions [state_num]
  float* d_jacobian_ops;     ///< 6x9 Jacobian matrices [ops_num * 54]
  float* d_areas;            ///< Per-face rest areas [ops_num]
  float* d_outer_rhs;        ///< Outer loop RHS [state_num]
  float* d_elastic_rhs;      ///< Elastic force accumulator [state_num]
  float* d_final_rhs;        ///< Combined RHS [state_num]
  float* d_solution;         ///< Linear solve output [state_num]

 public:
  /**
   * @brief Create GPU context for a cloth object
   *
   * Allocates device memory and uploads static topology data (faces, Jacobian
   * operators, areas). Dynamic data (state, RHS) is allocated but not
   * initialized.
   *
   * @param config Physical parameters
   * @param topology Static mesh-dependent quantities
   * @param F Face connectivity matrix (vnum x 3)
   * @param time_step Simulation time step
   * @return GpuClothSolverContext if allocation succeeds, std::nullopt on
   * failure
   */
  static std::optional<GpuClothSolverContext> create(
      const ClothConfig& config, const ClothTopology& topology,
      const RMatrixX3i& F, float time_step);

  /**
   * @brief Upload current state to GPU
   *
   * @param state Host-side vertex positions (Eigen vector, length state_num)
   */
  void upload_state(const Eigen::VectorXf& state);

  /**
   * @brief Upload outer loop RHS to GPU
   *
   * @param outer_rhs Host-side RHS vector (Eigen vector, length state_num)
   */
  void upload_outer_rhs(const Eigen::VectorXf& outer_rhs);

  /**
   * @brief Download solution from GPU
   *
   * @param solution Host-side output buffer (Eigen vector, length state_num)
   */
  void download_solution(Eigen::Ref<Eigen::VectorXf> solution);

  /**
   * @brief Execute GPU inner loop computation
   *
   * Launches CUDA kernels to:
   * 1. Compute elastic RHS contributions (per-face SVD projection)
   * 2. Combine with outer_rhs to form final RHS
   *
   * Note: This does NOT perform the linear solve step (which still uses CPU
   * CHOLMOD). It only replaces the per-face projection and RHS assembly.
   *
   * The combined RHS is stored in d_final_rhs and can be downloaded via
   * download_solution().
   */
  void compute_elastic_rhs();

  /**
   * @brief Destructor - frees all device memory
   */
  ~GpuClothSolverContext();

  // Disable copy (device pointers are not copyable)
  GpuClothSolverContext(const GpuClothSolverContext&) = delete;
  GpuClothSolverContext& operator=(const GpuClothSolverContext&) = delete;

  // Enable move
  GpuClothSolverContext(GpuClothSolverContext&&) noexcept;
  GpuClothSolverContext& operator=(GpuClothSolverContext&&) noexcept;

 private:
  GpuClothSolverContext() = default;

  /**
   * @brief Free all device memory allocations
   */
  void free_device_memory();
};

}  // namespace gpu
}  // namespace silk
