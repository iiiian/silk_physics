#include "gpu_solver_widget.hpp"

#include <imgui.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include "ui_console.hpp"

GpuSolverWidget::GpuSolverWidget(Context& ctx,
                                 std::function<void(SolverBackend)> onChange)
    : ctx_(ctx), n_change_(std::move(onChange)) {}

void GpuSolverWidget::set_backend(SolverBackend b) { backend_ = b; }

void GpuSolverWidget::draw() {
  ImGui::PushID(this);

  if (ImGui::CollapsingHeader((title_ + "##gpu_solver").c_str(),
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    bool useGpu = (backend_ == SolverBackend::GPU);

    if (ImGui::Checkbox("Use GPU solver##gpu_solver_checkbox", &useGpu)) {
      // Status Log
      if (useGpu) {
        backend_ = SolverBackend::GPU;
        UI_LOGI("[UI] Solver backend -> GPU");

      } else {
        backend_ = SolverBackend::CPU;
        UI_LOGI("[UI] Solver backend -> CPU");
      }
    }
  }

  // Pass an animated negative value, e.g. -1.0f * (float)ImGui::GetTime() is
  // the recommended value. Adjust the factor if you want to adjust the
  // animation speed. ImGui::ProgressBar(-1.0f * (float)ImGui::GetTime(),
  // ImVec2(0.0f, 0.0f), "Simulating..");
  // ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
  // ImGui::Text("");

  ImGui::PopID();
}
