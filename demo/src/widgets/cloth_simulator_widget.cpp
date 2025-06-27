#include "cloth_simulator_widget.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>
#include <cmath>

namespace py = polyscope;

void ClothSimulatorWidget::enter_sim_mode() {
  assert((ui_ctx_.ui_mode == UIMode::Normal));
  assert((engine_ctx_.V.rows() != 0));
  assert((engine_ctx_.F.rows() != 0));

  // update mesh any way
  cloth_cfg_.mesh = silk::Mesh(engine_ctx_.V, engine_ctx_.F);

  // convert selection to pinned group
  int selection_size = ui_ctx_.selection.size();
  cloth_cfg_.pinned_verts = Eigen::VectorXi::Zero(selection_size);
  int idx = 0;
  for (auto v : ui_ctx_.selection) {
    cloth_cfg_.pinned_verts(idx) = v;
    idx++;
  }

  silk::WorldResult result;
  if (cloth_handle_) {
    result = solver_.update_cloth(cloth_cfg_, *cloth_handle_);
    if (result != silk::WorldResult::Success) {
      SPDLOG_ERROR("Fail to enter cloth sim mode, Reason: {}",
                    silk::to_string(result));
      return;
    }
  } else {
    silk::Handle h;
    result = solver_.add_cloth(cloth_cfg_, h);
    if (result != silk::WorldResult::Success) {
      SPDLOG_ERROR("Fail to enter cloth sim mode, Reason: {}",
                    silk::to_string(result));
      return;
    }

    cloth_handle_ = h;
  }

  solver_.set_constant_acce_field({0, 0, -gravity_});
  solver_.set_max_iterations(solver_max_iter_);
  solver_.set_thread_num(solver_thread_num_);
  result = solver_.set_dt(1.0f / target_fps_);
  if (result != silk::WorldResult::Success) {
    SPDLOG_ERROR("Fail to enter cloth sim mode, Reason: {}",
                  silk::to_string(result));
    return;
  }
  result = solver_.set_low_freq_mode_num(solver_low_freq_mode_num_);
  if (result != silk::WorldResult::Success) {
    SPDLOG_ERROR("Fail to enter cloth sim mode, Reason: {}",
                  silk::to_string(result));
    return;
  }

  result = solver_.solver_init();
  if (result != silk::WorldResult::Success) {
    solver_.solver_reset();
    SPDLOG_ERROR("Fail to enter cloth sim mode: Reason: {}",
                  silk::to_string(result));
    return;
  }

  original_V = engine_ctx_.V;
  prev_update_time_ = std::chrono::steady_clock::now();
  prev_mouse_pos_ = ImGui::GetMousePos();
  ui_ctx_.help_text = CLOTH_SIM_MODE_HELP_TXT;

  ui_ctx_.ui_mode = UIMode::ClothSim;
  SPDLOG_INFO("Enter cloth sim mode");
}

void ClothSimulatorWidget::leave_sim_mode() {
  solver_.solver_reset();
  engine_ctx_.V = original_V;
  ui_ctx_.p_surface->updateVertexPositions(original_V);
  original_V = {};
  py::state::doDefaultMouseInteraction = true;
  ui_ctx_.help_text = NORMAL_MODE_HELP_TXT;

  ui_ctx_.ui_mode = UIMode::Normal;
  SPDLOG_INFO("Leave cloth sim mode");
}

void ClothSimulatorWidget::compute_cloth(float elapse_sec) {
  int substep = elapse_sec / solver_.get_dt() + 1;
  assert((substep >= 1));

  // convert selection to pinned group
  int selection_size = ui_ctx_.selection.size();
  Eigen::VectorXf pinned_positions = Eigen::VectorXf::Zero(3 * selection_size);
  int idx = 0;
  for (auto v : ui_ctx_.selection) {
    pinned_positions(Eigen::seqN(3 * idx, 3)) = engine_ctx_.V.row(v);
    idx++;
  }

  auto result =
      solver_.update_position_constrain(*cloth_handle_, pinned_positions);
  if (result != silk::WorldResult::Success) {
    SPDLOG_ERROR("Cloth solve fail, Reason: {}", silk::to_string(result));
    return;
  }

  for (int s = 0; s < substep; ++s) {
    auto result = solver_.step();
    if (result != silk::WorldResult::Success) {
      SPDLOG_ERROR("Cloth solve fail, Reason: {}", silk::to_string(result));
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
  Eigen::Vector3f eg_pos = engine_ctx_.V.row(*ui_ctx_.selection.begin());
  glm::vec3 pos(eg_pos(0), eg_pos(1), eg_pos(2));
  float plane_dist = glm::dot(camera.getLookDir(), pos - camera.getPosition());

  float fov_y = camera.getFoVVerticalDegrees();
  // TODO: use proper pi
  float span_y = plane_dist * std::tan(3.14 * fov_y / 360);
  float span_x = camera.getAspectRatioWidthOverHeight() * span_y;

  ImVec2 win_size = ImGui::GetMainViewport()->Size;
  float mouse_dx_ratio = mouse_dx / win_size.x;
  float mouse_dy_ratio = mouse_dy / win_size.y;

  glm::vec3 camera_right = camera.getRightDir();
  glm::vec3 camera_up = camera.getUpDir();
  glm::vec3 delta = mouse_dx_ratio * span_x * camera_right -
                    mouse_dy_ratio * span_y * camera_up;
  auto eg_delta = Eigen::Map<Eigen::Vector3f>(glm::value_ptr(delta));

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
    ImGui::DragInt("Solver Threads", &solver_thread_num_, 1.0f, 1, 24, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("thread number for local projection step");

    ImGui::DragInt("Target FPS", &target_fps_, 1.0f, 1, 500, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip(
        "FPS of cloth solver, large elastic stiffness require high FPS");

    ImGui::DragInt("Max Iterations", &solver_max_iter_, 1.0f, 1, 10, "%d",
                   ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("The higher the better the detail");

    ImGui::DragInt("Low Frequency Mode Number", &solver_low_freq_mode_num_,
                   1.0f, 5, 100, "%d", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("Might improve performance?");

    ImGui::DragFloat("Elastic Stiffness", &cloth_cfg_.elastic_stiffness, 1.0f,
                     0, 1e6, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("the in-plane cloth stiffness");

    ImGui::DragFloat("Bending Stiffness", &cloth_cfg_.bending_stiffness, 1.0f,
                     0, 1e6, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("the cloth bending stiffness");

    ImGui::DragFloat("Density", &cloth_cfg_.density, 1.0f, 1e-3, 1e2, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("the cloth density");

    ImGui::DragFloat("Gravity", &gravity_, 1.0f, 1, 100, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("gravity acceleration");

    // ImGui::Checkbox("Enable Collision (Pretty Buggy)",
    //                 &solver.enable_collision);
    // ImGui::DragFloat("Thickness", &solver.collision_thickness_, 1.0f, 0, 100,
    //                  "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
  }
  ImGui::EndDisabled();

  // Handle drag selection in cloth sim mode
  handle_drag_selection();

  // update cloth
  if (ui_ctx_.ui_mode == UIMode::ClothSim) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapse_sec = now - prev_update_time_;
    if (elapse_sec.count() >= solver_.get_dt()) {
      compute_cloth(elapse_sec.count());
      prev_update_time_ = std::chrono::steady_clock::now();
    }
  }

  return EventFlag::NoEvent;
}

void ClothSimulatorWidget::on_event(EventFlag events) {}
