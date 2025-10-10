#include "gpu_solver_widget.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>

#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>

GpuSolverWidget::GpuSolverWidget(Context& ctx,
                                 std::function<void(SolverBackend, SolverBackend)> onChange)
  : ctx_(ctx), on_change_(std::move(onChange)) {}

void GpuSolverWidget::set_backend(SolverBackend b) { backend_ = b; }

void GpuSolverWidget::draw() {
  if (ImGui::CollapsingHeader(title_.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
    bool useGpu = (backend_ == SolverBackend::GPU);
    if (ImGui::Checkbox("Use GPU solver", &useGpu)) {
      SolverBackend old = backend_;
      backend_ = useGpu ? SolverBackend::GPU : SolverBackend::CPU;
      spdlog::info("[UI] Solver backend -> {}", useGpu ? "GPU" : "CPU");
      if (on_change_) on_change_(old, backend_);
    }
  }
}




