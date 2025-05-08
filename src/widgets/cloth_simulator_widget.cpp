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
  // Only intercept left mouse button for dragging
  py::state::userCallback_mouseMove = [this](py::MouseButton button, float x, float y) {
    return button == py::MouseButton::Left;
  };
  spdlog::info("Enter cloth sim mode");
}

void ClothSimulatorWidget::leave_sim_mode() {
  solver.reset();
  engine_ctx_.V = original_V;
  ui_ctx_.p_surface->updateVertexPositions(original_V);
  original_V = {};
  ui_ctx_.ui_mode = UIMode::Normal;
  py::state::userCallback_mouseMove = nullptr;
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

void ClothSimulatorWidget::handle_drag_selection() {
  if (ui_ctx_.ui_mode != UIMode::ClothSim || ui_ctx_.selection.empty()) {
    return;
  }

  ImVec2 mouse_pos = ImGui::GetMousePos();
  bool mouse_moved = false;
  float delta_x = 0.0f;
  float delta_y = 0.0f;
  
  if (prev_mouse_pos_.x >= 0 && prev_mouse_pos_.y >= 0) {
    delta_x = mouse_pos.x - prev_mouse_pos_.x;
    delta_y = mouse_pos.y - prev_mouse_pos_.y;
    mouse_moved = (std::abs(delta_x) > 1 || std::abs(delta_y) > 1);
  }

  // Only handle left mouse button for dragging
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    if (!is_dragging_) {
      is_dragging_ = true;
    }

    if (is_dragging_ && mouse_moved) {
      // Get the camera view direction and up vector
      glm::vec3 view_dir = py::view::getCameraDirection();
      glm::vec3 up_dir = py::view::getCameraUp();
      glm::vec3 right_dir = glm::cross(view_dir, up_dir);
      
      // Scale movement based on mesh size
      float scale_factor = ui_ctx_.mesh_diag * 0.001f;
      float dx = delta_x * scale_factor;
      float dy = -delta_y * scale_factor; // Invert Y for screen coordinates
      
      // Apply movement to selected vertices
      for (int idx : ui_ctx_.selection) {
        if (idx < engine_ctx_.V.rows()) {
          engine_ctx_.V.row(idx) += dx * Eigen::RowVector3f(right_dir.x, right_dir.y, right_dir.z);
          engine_ctx_.V.row(idx) += dy * Eigen::RowVector3f(up_dir.x, up_dir.y, up_dir.z);
        }
      }
      
      // Update the mesh visualization
      ui_ctx_.p_surface->updateVertexPositions(engine_ctx_.V);
    }
  } else if (is_dragging_) {
    is_dragging_ = false;
  }

  prev_mouse_pos_ = mouse_pos;
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

  // Handle drag selection in cloth sim mode
  handle_drag_selection();

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
