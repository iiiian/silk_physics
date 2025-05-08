#include "cloth_simulator_widget.hpp"

#include <cassert>

void ClothSimulatorWidget::enter_sim_mode() {
  assert((engine_ctx_.V.rows() != 0));
  assert((engine_ctx_.F.rows() != 0));
  assert(!ui_ctx_.selection.empty());

  solver.pV_ = &engine_ctx_.V;
  solver.pF_ = &engine_ctx_.F;
  solver.pconstrain_set = &ui_ctx_.selection;
  solver.init();
}

void ClothSimulatorWidget::leave_sim_mode() {
  solver.reset();
  ui_ctx_.ui_mode = UIMode::Normal;
}

void ClothSimulatorWidget::compute_cloth() {
  // TODO: update constrain

  solver.solve();
}

ClothSimulatorWidget::ClothSimulatorWidget(UIContext& ui_context,
                                           EngineContext& engine_context)
    : ui_ctx_(ui_context), engine_ctx_(engine_context) {};

EventFlag ClothSimulatorWidget::draw() {
  ImGui::BeginDisabled(!ui_ctx_.p_surface ||
                       !(ui_ctx_.ui_mode == UIMode::Normal ||
                         ui_ctx_.ui_mode == UIMode::ClothSim));

  if (ImGui::CollapsingHeader("Cloth Simulation",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button(ui_ctx_.ui_mode == UIMode::Paint ? "Stop Simulation"
                                                       : "Start Simulation")) {
      if (ui_ctx_.ui_mode == UIMode::ClothSim) {
        leave_sim_mode();
      } else {
        enter_sim_mode();
      }
    }

    // During simulation these parameters can't be adjuested
    ImGui::BeginDisabled(ui_ctx_.ui_mode == UIMode::ClothSim);
    ImGui::SliderInt("Target FPS", &target_fps_, 1, 120, "%.4d",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Elastic Stiffness", &solver.elastic_stiffness_, 0, 1e4,
                       "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Bending Stiffness", &solver.bending_stiffness_, 0, 1e4,
                       "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Density", &solver.density_, 1e-3, 1e2, "%.3f",
                       ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Zero prune threshold", &solver.zero_prune_threshold_, 0,
                       1e-2, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
  }
  ImGui::EndDisabled();

  // update cloth
  if (ui_ctx_.ui_mode == UIMode::ClothSim) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapse_sec = now - prev_update_time;
    if (elapse_sec.count() >= solver.dt_) {
      compute_cloth();
    }
  }

  return EventFlag::NoEvent;
}

void ClothSimulatorWidget::on_event(EventFlag events) {}
