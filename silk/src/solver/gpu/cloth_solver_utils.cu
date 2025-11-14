#include "cloth_solver_utils.hpp"

#include "logger.hpp"
#include "solver/cpu/cholmod_utils.hpp"

namespace silk {
namespace gpu {

bool compute_cloth_inner_loop_gpu(
    const ClothConfig& config, const RMatrixX3i& F,
    const ClothTopology& topology,
    const CpuClothSolverContext& cpu_solver_context,
    GpuClothSolverContext& gpu_solver_context,
    Eigen::Ref<const Eigen::VectorXf> state,
    Eigen::Ref<const Eigen::VectorXf> outer_rhs,
    Eigen::Ref<Eigen::VectorXf> solution) {

  auto& s = cpu_solver_context;
  auto& gpu = gpu_solver_context;

  try {
    // --- 1. Upload current state and outer RHS to GPU ---
    gpu.upload_state(state);
    gpu.upload_outer_rhs(outer_rhs);

    // --- 2. Compute elastic RHS on GPU ---
    // This replaces the CPU TBB parallel_for loop (lines 234-274)
    // The GPU kernel performs:
    // - Per-face SVD decomposition
    // - Singular value clamping
    // - Elastic force computation
    // - Atomic accumulation to global RHS
    gpu.compute_elastic_rhs();

    // --- 3. Download combined RHS (outer + elastic) ---
    Eigen::VectorXf rhs(state.size());
    gpu.download_solution(rhs);

    // --- 4. Global linear solve using CPU CHOLMOD ---
    // This part remains on CPU for now. A future optimization could move this
    // to GPU using cuSPARSE/cuSOLVER.
    cholmod_dense rhs_view = cholmod_raii::make_cholmod_dense_view(rhs);
    auto& L = (s.has_barrier_constrain) ? s.LB : s.L;

    cholmod_raii::CholmodDense cholmod_solution =
        cholmod_solve(CHOLMOD_A, L, &rhs_view, cholmod_raii::common);
    if (cholmod_solution.is_empty()) {
      SPDLOG_ERROR("GPU inner loop: CHOLMOD solve failed");
      return false;
    }

    solution = cholmod_raii::make_eigen_dense_vector_view(cholmod_solution);

    return true;

  } catch (const std::exception& e) {
    SPDLOG_ERROR("GPU inner loop failed: {}", e.what());
    return false;
  }
}

}  // namespace gpu
}  // namespace silk
