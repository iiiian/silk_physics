#include "cloth_simulator_widget.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <numbers>

namespace py = polyscope;
namespace eg = Eigen;

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
  prev_mouse_pos_ = ImGui::GetMousePos();
  ui_ctx_.help_text = CLOTH_SIM_MODE_HELP_TXT;

  ui_ctx_.ui_mode = UIMode::ClothSim;
  spdlog::info("Enter cloth sim mode");
}

void ClothSimulatorWidget::leave_sim_mode() {
  solver.reset();
  engine_ctx_.V = original_V;
  ui_ctx_.p_surface->updateVertexPositions(original_V);
  original_V = {};
  py::state::doDefaultMouseInteraction = true;
  ui_ctx_.help_text = NORMAL_MODE_HELP_TXT;

  ui_ctx_.ui_mode = UIMode::Normal;
  spdlog::info("Leave cloth sim mode");
}

void ClothSimulatorWidget::compute_cloth(float elapse_sec) {
  int substep = elapse_sec / solver.dt_ + 1;
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
  float mouse_dx = mouse_pos.x - prev_mouse_pos_.x;
  float mouse_dy = mouse_pos.y - prev_mouse_pos_.y;

  bool has_selection = !ui_ctx_.selection.empty();
  bool is_left_click = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool is_ctrl = ImGui::GetIO().KeyCtrl;
  bool is_moving = (std::abs(mouse_dx) > 1 || std::abs(mouse_dy) > 1);
  if (!(has_selection && is_left_click && is_ctrl && is_moving)) {
    py::state::doDefaultMouseInteraction = true;
    prev_mouse_pos_ = mouse_pos;
    return;
  }

  // hijack mouse handling
  py::state::doDefaultMouseInteraction = false;

  py::CameraParameters camera = py::view::getCameraParametersForCurrentView();
  // choose a random selected vertex to calculate distance
  eg::Vector3f eg_pos = engine_ctx_.V.row(*ui_ctx_.selection.begin());
  glm::vec3 pos(eg_pos(0), eg_pos(1), eg_pos(2));
  float plane_dist = glm::dot(camera.getLookDir(), pos - camera.getPosition());

  float fov_y = camera.getFoVVerticalDegrees();
  float span_y = plane_dist * std::tan(std::numbers::pi * fov_y / 360);
  float span_x = camera.getAspectRatioWidthOverHeight() * span_y;

  ImVec2 win_size = ImGui::GetMainViewport()->Size;
  float mouse_dx_ratio = mouse_dx / win_size.x;
  float mouse_dy_ratio = mouse_dy / win_size.y;

  glm::vec3 camera_right = camera.getRightDir();
  glm::vec3 camera_up = camera.getUpDir();
  glm::vec3 delta = mouse_dx_ratio * span_x * camera_right -
                    mouse_dy_ratio * span_y * camera_up;
  auto eg_delta = eg::Map<eg::Vector3f>(glm::value_ptr(delta));

  // update both ui and engine mesh
  auto& mesh_verts = ui_ctx_.p_surface->vertexPositions;
  mesh_verts.ensureHostBufferAllocated();
  for (int idx : ui_ctx_.selection) {
    assert((idx < engine_ctx_.V.rows()));
    assert((idx < mesh_verts.size()));

    engine_ctx_.V.row(idx) += eg_delta;
    mesh_verts.data[idx] += delta;
  }
  mesh_verts.markHostBufferUpdated();

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
    ImGui::DragInt("Solver Threads", &solver.thread_num_, 1.0f, 1, 24, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragInt("Target FPS", &target_fps_, 1.0f, 1, 500, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Elastic Stiffness", &solver.elastic_stiffness_, 1.0f, 0,
                     1e6, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Bending Stiffness", &solver.bending_stiffness_, 1.0f, 0,
                     1e6, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Density", &solver.density_, 1.0f, 1e-3, 1e2, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("Gravity", &gravity, 1.0f, 1, 100, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::Checkbox("Enable Collision (Pretty Buggy)",
                    &solver.enable_collision);
    ImGui::DragFloat("Thickness", &solver.collision_thickness_, 1.0f, 0, 100,
                     "%.3f", ImGuiSliderFlags_AlwaysClamp);
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
