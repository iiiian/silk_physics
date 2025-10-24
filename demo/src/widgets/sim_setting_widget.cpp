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
  }

  ImGui::EndDisabled();
}
