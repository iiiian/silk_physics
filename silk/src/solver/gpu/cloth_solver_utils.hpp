#pragma once

/**
 * @file cloth_solver_utils.hpp (GPU version)
 * @brief GPU-accelerated cloth solver utilities
 *
 * This header provides GPU versions of the cloth solver inner loop functions,
 * maintaining API compatibility with the CPU version while using CUDA
 * acceleration for the elastic RHS computation.
 */

#include <Eigen/Core>

#include "cloth_topology.hpp"
#include "eigen_alias.hpp"
#include "gpu_cloth_solver_context.hpp"
#include "silk/silk.hpp"
#include "solver/cpu/cloth_solver_context.hpp"

namespace silk {
namespace gpu {

/**
 * @brief GPU-accelerated inner loop of projective dynamics cloth solver
 *
 * This function replaces the CPU inner loop's per-face elastic projection with
 * GPU acceleration. It performs:
 * 1. Upload current state and outer_rhs to GPU
 * 2. Compute elastic RHS via CUDA kernel (per-face SVD projection)
 * 3. Combine with outer_rhs
 * 4. Download combined RHS
 * 5. Perform linear solve using CPU CHOLMOD (H * x = rhs)
 *
 * The elastic projection step (lines 234-274 in CPU version) is moved to GPU,
 * while the linear solve remains on CPU for now.
 *
 * @param config Physical parameters
 * @param F Face connectivity matrix
 * @param topology Static mesh-dependent quantities
 * @param cpu_solver_context CPU context (for CHOLMOD linear solve)
 * @param gpu_solver_context GPU context (device memory and kernels)
 * @param state Current vertex positions [state_num]
 * @param outer_rhs RHS from outer loop [state_num]
 * @param solution Output: computed vertex update [state_num]
 * @return true on success, false if linear solve fails
 */
bool compute_cloth_inner_loop_gpu(
    const ClothConfig& config, const RMatrixX3i& F,
    const ClothTopology& topology,
    const CpuClothSolverContext& cpu_solver_context,
    GpuClothSolverContext& gpu_solver_context,
    Eigen::Ref<const Eigen::VectorXf> state,
    Eigen::Ref<const Eigen::VectorXf> outer_rhs,
    Eigen::Ref<Eigen::VectorXf> solution);

}  // namespace gpu
}  // namespace silk
