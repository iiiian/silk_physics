#include "cloth_simulator_widget.hpp"

#include <spdlog/spdlog.h>

#include <cassert>

void ClothSimulatorWidget::enter_sim_mode() {
  assert((ui_ctx_.ui_mode == UIMode::Normal));
  assert((engine_ctx_.V.rows() != 0));
  assert((engine_ctx_.F.rows() != 0));
  assert(!ui_ctx_.selection.empty());

  solver.pV_ = &engine_ctx_.V;
  solver.pF_ = &engine_ctx_.F;
  solver.pconstrain_set = &ui_ctx_.selection;
  solver.dt_ = 1.0f / target_fps_;
  if (!solver.init()) {
    solver.reset();
    spdlog::error("Fail to enter cloth sim mode");
    return;
  }

  original_V = engine_ctx_.V;
  prev_update_time = std::chrono::steady_clock::now();
  ui_ctx_.ui_mode = UIMode::ClothSim;
  spdlog::info("Enter cloth sim mode");
}

void ClothSimulatorWidget::leave_sim_mode() {
  solver.reset();
  engine_ctx_.V = original_V;
  ui_ctx_.p_surface->updateVertexPositions(original_V);
  original_V = {};
  ui_ctx_.ui_mode = UIMode::Normal;
  spdlog::info("Leave cloth sim mode");
}

void ClothSimulatorWidget::compute_cloth() {
  // TODO: update constrain

  if (!solver.solve()) {
    spdlog::error("Cloth solve fail");
    leave_sim_mode();
  }

  ui_ctx_.p_surface->updateVertexPositions(engine_ctx_.V);
  spdlog::info("cloth postition update");
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
    if (ImGui::Button(ui_ctx_.ui_mode == UIMode::ClothSim
                          ? "Stop Simulation"
                          : "Start Simulation")) {
      if (ui_ctx_.ui_mode == UIMode::ClothSim) {
        leave_sim_mode();
      } else {
        enter_sim_mode();
      }
    }

    // During simulation these parameters can't be adjuested
    ImGui::BeginDisabled(ui_ctx_.ui_mode == UIMode::ClothSim);
    ImGui::SliderInt("Target FPS", &target_fps_, 1, 120, "%d",
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
      prev_update_time = std::chrono::steady_clock::now();
    }
  }

  return EventFlag::NoEvent;
}

void ClothSimulatorWidget::on_event(EventFlag events) {}
