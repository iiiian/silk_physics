#include "sim_setting_widget.hpp"

SimSettingWidget::SimSettingWidget(Context& context) : ctx_(context) {}

void SimSettingWidget::draw() {
  ImGui::BeginDisabled((ctx_.ui_mode != UIMode::Normal));

  if (ImGui::CollapsingHeader("Simulation Settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::InputInt("Target FPS", &target_fps_)) {
      ctx_.global_config.dt = 1.0f / target_fps_;
    }

    ImGui::InputFloat("Acceleration (X)", &ctx_.global_config.acceleration_x);
    ImGui::InputFloat("Acceleration (Y)", &ctx_.global_config.acceleration_y);
    ImGui::InputFloat("Acceleration (Z)", &ctx_.global_config.acceleration_z);

    ImGui::InputInt("Max Solver Iterations", &ctx_.global_config.max_iteration);

    ImGui::InputInt("Low Freqency Mode Num", &ctx_.global_config.r);

    ImGui::InputFloat("CCD Walkback", &ctx_.global_config.ccd_walkback);

    ImGui::InputFloat("TOI Tolerance", &ctx_.global_config.toi_tolerance);

    ImGui::InputInt("TOI Refine Iterations",
                    &ctx_.global_config.toi_refine_iteration);

    ImGui::InputFloat("Eps", &ctx_.global_config.eps);
  }

  ImGui::EndDisabled();
}
