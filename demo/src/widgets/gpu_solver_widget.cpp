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
    bool use_gpu = (backend_ == SolverBackend::GPU);

    if (ImGui::Checkbox("Use GPU solver##gpu_solver_checkbox", &use_gpu)) {
      // Status Log
      if (use_gpu) {
        backend_ = SolverBackend::GPU;
        ui_info("[UI] Solver backend -> GPU");
      } else {
        backend_ = SolverBackend::CPU;
        ui_info("[UI] Solver backend -> CPU");
      }
    }
  }

  ImGui::PopID();
}
