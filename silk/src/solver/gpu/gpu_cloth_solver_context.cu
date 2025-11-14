#include "gpu_cloth_solver_context.hpp"

#include <cuda_runtime.h>

#include <iostream>
#include <stdexcept>

#include "cloth_solver_kernels.cuh"
#include "logger.hpp"

namespace silk {
namespace gpu {

// --- CUDA Error Checking Macro ---
#define CHECK_CUDA_ERROR(val) check_cuda((val), #val, __FILE__, __LINE__)

static void check_cuda(cudaError_t result, char const* const func,
                       const char* const file, int const line) {
  if (result != cudaSuccess) {
    std::string error_msg = std::string("CUDA Error at ") + file + ":" +
                            std::to_string(line) + " code=" +
                            std::to_string(static_cast<unsigned int>(result)) +
                            " \"" + func +
                            "\" : " + cudaGetErrorString(result);
    SPDLOG_ERROR(error_msg);
    throw std::runtime_error(error_msg);
  }
}

// ====================================================================
// GpuClothSolverContext Implementation
// ====================================================================

std::optional<GpuClothSolverContext> GpuClothSolverContext::create(
    const ClothConfig& config, const ClothTopology& topology,
    const RMatrixX3i& F, float time_step) {
  GpuClothSolverContext ctx;

  ctx.dt = time_step;
  ctx.state_num = topology.mass.size() * 3;  // 3 * vnum
  ctx.ops_num = topology.jacobian_ops.size();
  ctx.elastic_stiffness = config.elastic_stiffness;

  SPDLOG_INFO("Creating GPU cloth solver context:");
  SPDLOG_INFO("  state_num = {}", ctx.state_num);
  SPDLOG_INFO("  ops_num (faces) = {}", ctx.ops_num);
  SPDLOG_INFO("  dt = {}", ctx.dt);

  try {
    // --- 1. Allocate Device Memory ---
    CHECK_CUDA_ERROR(cudaMalloc(&ctx.d_F, ctx.ops_num * 3 * sizeof(int)));
    CHECK_CUDA_ERROR(cudaMalloc(&ctx.d_state, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMalloc(&ctx.d_jacobian_ops, ctx.ops_num * 54 * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&ctx.d_areas, ctx.ops_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMalloc(&ctx.d_outer_rhs, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMalloc(&ctx.d_elastic_rhs, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMalloc(&ctx.d_final_rhs, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMalloc(&ctx.d_solution, ctx.state_num * sizeof(float)));

    SPDLOG_INFO("  Device memory allocated successfully");

    // --- 2. Upload Static Topology Data ---

    // Face indices (convert from Eigen matrix to flat array)
    std::vector<int> h_F(ctx.ops_num * 3);
    for (int i = 0; i < ctx.ops_num; ++i) {
      h_F[i * 3 + 0] = F(i, 0);
      h_F[i * 3 + 1] = F(i, 1);
      h_F[i * 3 + 2] = F(i, 2);
    }
    CHECK_CUDA_ERROR(cudaMemcpy(ctx.d_F, h_F.data(),
                                 ctx.ops_num * 3 * sizeof(int),
                                 cudaMemcpyHostToDevice));

    // Jacobian operators (6x9 matrices stored row-major)
    std::vector<float> h_jacobian_ops(ctx.ops_num * 54);
    for (int i = 0; i < ctx.ops_num; ++i) {
      const auto& jac = topology.jacobian_ops[i];
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 9; ++col) {
          h_jacobian_ops[i * 54 + row * 9 + col] = jac(row, col);
        }
      }
    }
    CHECK_CUDA_ERROR(cudaMemcpy(ctx.d_jacobian_ops, h_jacobian_ops.data(),
                                 ctx.ops_num * 54 * sizeof(float),
                                 cudaMemcpyHostToDevice));

    // Per-face areas
    std::vector<float> h_areas(ctx.ops_num);
    for (int i = 0; i < ctx.ops_num; ++i) {
      h_areas[i] = topology.area(i);
    }
    CHECK_CUDA_ERROR(cudaMemcpy(ctx.d_areas, h_areas.data(),
                                 ctx.ops_num * sizeof(float),
                                 cudaMemcpyHostToDevice));

    SPDLOG_INFO("  Static topology uploaded to device");

    // --- 3. Initialize dynamic buffers to zero ---
    CHECK_CUDA_ERROR(cudaMemset(ctx.d_state, 0, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMemset(ctx.d_outer_rhs, 0, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMemset(ctx.d_elastic_rhs, 0, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMemset(ctx.d_final_rhs, 0, ctx.state_num * sizeof(float)));
    CHECK_CUDA_ERROR(
        cudaMemset(ctx.d_solution, 0, ctx.state_num * sizeof(float)));

    SPDLOG_INFO("GPU cloth solver context created successfully");

    return ctx;

  } catch (const std::exception& e) {
    SPDLOG_ERROR("Failed to create GPU cloth solver context: {}", e.what());
    ctx.free_device_memory();
    return std::nullopt;
  }
}

void GpuClothSolverContext::upload_state(const Eigen::VectorXf& state) {
  if (state.size() != state_num) {
    throw std::runtime_error("State size mismatch: expected " +
                             std::to_string(state_num) + ", got " +
                             std::to_string(state.size()));
  }

  CHECK_CUDA_ERROR(cudaMemcpy(d_state, state.data(),
                               state_num * sizeof(float),
                               cudaMemcpyHostToDevice));
}

void GpuClothSolverContext::upload_outer_rhs(const Eigen::VectorXf& outer_rhs) {
  if (outer_rhs.size() != state_num) {
    throw std::runtime_error("Outer RHS size mismatch: expected " +
                             std::to_string(state_num) + ", got " +
                             std::to_string(outer_rhs.size()));
  }

  CHECK_CUDA_ERROR(cudaMemcpy(d_outer_rhs, outer_rhs.data(),
                               state_num * sizeof(float),
                               cudaMemcpyHostToDevice));
}

void GpuClothSolverContext::download_solution(
    Eigen::Ref<Eigen::VectorXf> solution) {
  if (solution.size() != state_num) {
    throw std::runtime_error("Solution size mismatch: expected " +
                             std::to_string(state_num) + ", got " +
                             std::to_string(solution.size()));
  }

  CHECK_CUDA_ERROR(cudaMemcpy(solution.data(), d_final_rhs,
                               state_num * sizeof(float),
                               cudaMemcpyDeviceToHost));
}

void GpuClothSolverContext::compute_elastic_rhs() {
  // --- 1. Zero out elastic RHS accumulator ---
  CHECK_CUDA_ERROR(cudaMemset(d_elastic_rhs, 0, state_num * sizeof(float)));

  // --- 2. Launch elastic RHS kernel ---
  launch_compute_elastic_rhs_kernel(ops_num, d_F, d_state, d_jacobian_ops,
                                    d_areas, elastic_stiffness, d_elastic_rhs);

  // --- 3. Combine outer_rhs + elastic_rhs -> final_rhs ---
  launch_add_vectors_kernel(state_num, d_outer_rhs, d_elastic_rhs, d_final_rhs);

  // --- 4. Synchronize to ensure kernels complete ---
  CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void GpuClothSolverContext::free_device_memory() {
  if (d_F) cudaFree(d_F);
  if (d_state) cudaFree(d_state);
  if (d_jacobian_ops) cudaFree(d_jacobian_ops);
  if (d_areas) cudaFree(d_areas);
  if (d_outer_rhs) cudaFree(d_outer_rhs);
  if (d_elastic_rhs) cudaFree(d_elastic_rhs);
  if (d_final_rhs) cudaFree(d_final_rhs);
  if (d_solution) cudaFree(d_solution);

  d_F = nullptr;
  d_state = nullptr;
  d_jacobian_ops = nullptr;
  d_areas = nullptr;
  d_outer_rhs = nullptr;
  d_elastic_rhs = nullptr;
  d_final_rhs = nullptr;
  d_solution = nullptr;
}

GpuClothSolverContext::~GpuClothSolverContext() { free_device_memory(); }

GpuClothSolverContext::GpuClothSolverContext(GpuClothSolverContext&& other) noexcept
    : dt(other.dt),
      state_num(other.state_num),
      ops_num(other.ops_num),
      elastic_stiffness(other.elastic_stiffness),
      d_F(other.d_F),
      d_state(other.d_state),
      d_jacobian_ops(other.d_jacobian_ops),
      d_areas(other.d_areas),
      d_outer_rhs(other.d_outer_rhs),
      d_elastic_rhs(other.d_elastic_rhs),
      d_final_rhs(other.d_final_rhs),
      d_solution(other.d_solution) {
  // Null out source to avoid double-free
  other.d_F = nullptr;
  other.d_state = nullptr;
  other.d_jacobian_ops = nullptr;
  other.d_areas = nullptr;
  other.d_outer_rhs = nullptr;
  other.d_elastic_rhs = nullptr;
  other.d_final_rhs = nullptr;
  other.d_solution = nullptr;
}

GpuClothSolverContext& GpuClothSolverContext::operator=(
    GpuClothSolverContext&& other) noexcept {
  if (this != &other) {
    free_device_memory();

    dt = other.dt;
    state_num = other.state_num;
    ops_num = other.ops_num;
    elastic_stiffness = other.elastic_stiffness;
    d_F = other.d_F;
    d_state = other.d_state;
    d_jacobian_ops = other.d_jacobian_ops;
    d_areas = other.d_areas;
    d_outer_rhs = other.d_outer_rhs;
    d_elastic_rhs = other.d_elastic_rhs;
    d_final_rhs = other.d_final_rhs;
    d_solution = other.d_solution;

    other.d_F = nullptr;
    other.d_state = nullptr;
    other.d_jacobian_ops = nullptr;
    other.d_areas = nullptr;
    other.d_outer_rhs = nullptr;
    other.d_elastic_rhs = nullptr;
    other.d_final_rhs = nullptr;
    other.d_solution = nullptr;
  }
  return *this;
}

}  // namespace gpu
}  // namespace silk
