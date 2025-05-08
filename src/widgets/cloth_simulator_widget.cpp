#include "cloth_simulator_widget.hpp"

#include <spdlog/spdlog.h>

#include <cassert>

void ClothSimulatorWidget::enter_sim_mode() {
  assert((ui_ctx_.ui_mode == UIMode::Normal));
  assert((engine_ctx_.V.rows() != 0));
  assert((engine_ctx_.F.rows() != 0));

  solver.pV_ = &engine_ctx_.V;
  solver.pF_ = &engine_ctx_.F;
  solver.pconstrain_set = &ui_ctx_.selection;
  solver.dt_ = 1.0f / target_fps_;
  solver.constant_acce_field_ = {0, 0, -gravity};
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

void ClothSimulatorWidget::compute_cloth(float elapse_sec) {
  // TODO: update constrain

  int substep = elapse_sec / solver.dt_;
  assert((substep >= 1));

  for (int s = 0; s < substep; ++s) {
    if (!solver.solve()) {
      spdlog::error("Cloth solve fail");
      leave_sim_mode();
    }
  }

  ui_ctx_.p_surface->updateVertexPositions(engine_ctx_.V);
}

ClothSimulatorWidget::ClothSimulatorWidget(UIContext& ui_context,
                                           EngineContext& engine_context)
    : ui_ctx_(ui_context), engine_ctx_(engine_context) {};

EventFlag ClothSimulatorWidget::draw() {
  ImGui::BeginDisabled(!ui_ctx_.p_surface ||
                       !(ui_ctx_.ui_mode == UIMode::Normal ||
                         ui_ctx_.ui_mode == UIMode::ClothSim));
  // debug
  // static bool tt = true;
  // if (tt) {
  //   enter_sim_mode();
  //   tt = false;
  // }

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
    ImGui::DragInt("Target FPS", &target_fps_, 1.0f, 1, 120, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Elastic Stiffness", &solver.elastic_stiffness_, 1.0f, 0,
                     1e2, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Bending Stiffness", &solver.bending_stiffness_, 1.0f, 0,
                     1e2, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Density", &solver.density_, 1.0f, 1e-3, 1e2, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Gravity", &gravity, 1.0f, 1, 100, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
  }
  ImGui::EndDisabled();

  // update cloth
  if (ui_ctx_.ui_mode == UIMode::ClothSim) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapse_sec = now - prev_update_time;
    if (elapse_sec.count() >= solver.dt_) {
      compute_cloth(elapse_sec.count());
      prev_update_time = std::chrono::steady_clock::now();
    }
  }

  return EventFlag::NoEvent;
}

void ClothSimulatorWidget::on_event(EventFlag events) {}
