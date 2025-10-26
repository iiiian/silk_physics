#include "gpu_solver_widget.hpp"

#include <imgui.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include "ui_console.hpp"

GpuSolverWidget::GpuSolverWidget(Context& ctx) : ctx_(ctx) {}

void GpuSolverWidget::draw() {
  ImGui::PushID(this);

  if (ImGui::CollapsingHeader("Solver##gpu_solver",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    // Get current backend from World
    silk::SolverBackend current_backend = ctx_.silk_world.get_solver_backend();
    bool use_gpu = (current_backend == silk::SolverBackend::GPU);

    if (ImGui::Checkbox("Use GPU solver##gpu_solver_checkbox", &use_gpu)) {
      // Update backend selection
      silk::SolverBackend new_backend = use_gpu ? silk::SolverBackend::GPU
                                                 : silk::SolverBackend::CPU;

      auto result = ctx_.silk_world.set_solver_backend(new_backend);

      if (result) {
        backend_ = new_backend;
        // Status Log
        if (use_gpu) {
          ui_info("[UI] Solver backend switched to GPU (CUDA Jacobi)");
        } else {
          ui_info("[UI] Solver backend switched to CPU (CHOLMOD Cholesky)");
        }
      } else {
        ui_error("[UI] Failed to switch solver backend: " + result.to_string());
      }
    }
  }

  ImGui::PopID();
}
