#include "sim_setting_widget.hpp"

SimSettingWidget::SimSettingWidget(Context& context) : ctx_(context) {}

void SimSettingWidget::draw() {
  ImGui::BeginDisabled((ctx_.ui_mode != UIMode::Normal));

  if (ImGui::CollapsingHeader("Simulation Settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ctx_.global_config.dt != 0) {
      target_fps_ = 1.0f / ctx_.global_config.dt;
    }
    if (ImGui::InputInt("Target FPS", &target_fps_)) {
      if (ctx_.global_config.dt != 0) {
        ctx_.global_config.dt = 1.0f / target_fps_;
      }
    }

    ImGui::InputFloat("Acceleration (X)", &ctx_.global_config.acceleration_x);
    ImGui::InputFloat("Acceleration (Y)", &ctx_.global_config.acceleration_y);
    ImGui::InputFloat("Acceleration (Z)", &ctx_.global_config.acceleration_z);

    ImGui::InputInt("Max Solver Outer Iterations",
                    &ctx_.global_config.max_outer_iteration);
    ImGui::InputInt("Max Solver Inner Iterations",
                    &ctx_.global_config.max_inner_iteration);

    ImGui::InputFloat("Linear Solver Absolute Tolerance",
                      &ctx_.global_config.linear_solver_abs_tol, 0.0f, 0.0f,
                      "%.3e");
    ImGui::InputFloat("Linear Solver Minimum Relative Tolerance",
                      &ctx_.global_config.linear_solver_rel_tol_min, 0.0f, 0.0f,
                      "%.3e");
    ImGui::InputFloat("Linear Solver Maximum Relative Tolerance",
                      &ctx_.global_config.linear_solver_rel_tol_max, 0.0f, 0.0f,
                      "%.3e");
    ImGui::InputFloat("Initial Linear Solver Relative Tolerance",
                      &ctx_.global_config.initial_linear_rel_tol, 0.0f, 0.0f,
                      "%.3e");
    ImGui::InputFloat("ADMM Absolute Tolerance",
                      &ctx_.global_config.admm_abs_tol, 0.0f, 0.0f, "%.3e");
    ImGui::InputFloat("ADMM Relative Tolerance",
                      &ctx_.global_config.admm_rel_tol, 0.0f, 0.0f, "%.3e");
  }

  ImGui::EndDisabled();
}
