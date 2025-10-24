#include "simulator_widget.hpp"

#include <polyscope/pick.h>
#include <polyscope/polyscope.h>
#include <portable-file-dialogs.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>

#include "../alembic_writer.hpp"

namespace py = polyscope;

void SimulatorWidget::enter_sim_mode() {
  silk::Result r = ctx_.silk_world.solver_reset();
  if (!r) {
    spdlog::error("Solver reset fail. Error: {}", r.to_string());
  }

  // init all objects via IObject
  for (auto& obj : ctx_.objects) {
    if (!obj->init_sim()) {
      spdlog::error("init_sim failed for object '{}'", obj->get_name());
      return;
    }
  }

  r = ctx_.silk_world.set_global_config(ctx_.global_config);
  if (!r) {
    spdlog::error("Set global config failed. Error: {}", r.to_string());
    return;
  }

  prev_update_time_ = std::chrono::steady_clock::now();
  sim_time_ = 0.0f;
  ctx_.ui_mode = UIMode::Sim;
  spdlog::info("Enter sim mode");
}

void SimulatorWidget::leave_sim_mode() {
  for (auto& obj : ctx_.objects) {
    if (!obj->exit_sim()) {
      spdlog::error("Object {} fail to exit simulation mode", obj->get_name());
    }
  }

  py::state::doDefaultMouseInteraction = true;
  ctx_.ui_mode = UIMode::Normal;
  spdlog::info("Leave sim mode");
}

void SimulatorWidget::solver_step(int substep) {
  // pre-step hooks
  for (auto& pobj : ctx_.objects) {
    if (!pobj->sim_step_pre()) {
      spdlog::info("Object {} fails to compute pre-simulation step.",
                   pobj->get_name());
      return;
    }
  }

  for (int i = 0; i < substep; ++i) {
    silk::Result r = ctx_.silk_world.solver_step();
    if (!r) {
      spdlog::error("Solver step fail. Error: {}", r.to_string());
      leave_sim_mode();
      return;
    }
  }

  // post-step hooks
  sim_time_ += substep * ctx_.global_config.dt;
  for (auto& pobj : ctx_.objects) {
    if (!pobj->sim_step_post(sim_time_)) {
      spdlog::info("Object {} fails to compute post-simulation step.",
                   pobj->get_name());
      return;
    }
  }
}

SimulatorWidget::SimulatorWidget(Context& context) : ctx_(context) {}

void SimulatorWidget::draw() {
  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Simulation FPS: %f", sim_fps_);
    ImGui::Text("Simulation Time: %f", sim_time_);

    ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal &&
                         ctx_.ui_mode != UIMode::Sim);
    if (ImGui::Button(ctx_.ui_mode == UIMode::Sim ? "Stop Simulation"
                                                  : "Start Simulation")) {
      if (ctx_.ui_mode == UIMode::Sim) {
        leave_sim_mode();
      } else {
        enter_sim_mode();
      }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();

    bool can_export_scene =
        (ctx_.ui_mode == UIMode::Normal) && !ctx_.objects.empty();
    ImGui::BeginDisabled(!can_export_scene);
    if (ImGui::Button("Export Simulation")) {
      std::vector<std::string> filters = {"Alembic Files", "*.abc"};

      std::string destination =
          pfd::save_file("Export Simulation", "scene export.abc", filters,
                         pfd::opt::force_overwrite)
              .result();

      if (!destination.empty()) {
        std::filesystem::path export_path(destination);
        if (!write_scene(export_path, ctx_.objects)) {
          spdlog::error("Failed to export scene to {}", export_path.string());
        } else {
          spdlog::info("Exported scene to {}", export_path.string());
        }
      }
    }
    ImGui::EndDisabled();
  }

  if (ctx_.ui_mode == UIMode::Sim) {
    handle_pin_dragging();

    using std::chrono::microseconds;
    using std::chrono::steady_clock;

    auto now = steady_clock::now();
    double elapsed_us =
        std::chrono::duration_cast<microseconds>(now - prev_update_time_)
            .count();

    double dt_us = 1e6f * ctx_.global_config.dt;
    int substep = static_cast<int>(std::floor(elapsed_us / dt_us));
    if (substep <= 0) {
      return;
    }

    spdlog::info("Solver step {}", substep);

    auto s0 = steady_clock::now();
    solver_step(substep);
    auto s1 = steady_clock::now();

    double sim_us = std::chrono::duration_cast<microseconds>(s1 - s0).count();
    if (sim_us <= 0.0f) {
      sim_us = 1.0f;
    }
    sim_fps_ = static_cast<float>(substep * 1e6f / sim_us);

    prev_update_time_ = steady_clock::now();
  }
}

void SimulatorWidget::handle_pin_dragging() {
  ImVec2 mouse_pos = ImGui::GetMousePos();
  bool is_left_click = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
  bool is_left_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool is_ctrl = ImGui::GetIO().KeyCtrl;

  float mouse_dx = mouse_pos.x - prev_mouse_pos_.x;
  float mouse_dy = mouse_pos.y - prev_mouse_pos_.y;
  prev_mouse_pos_ = mouse_pos;

  // On initial ctrl+click: determine drag object via picking
  if (is_left_click && is_ctrl) {
    py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});
    drag_object_ = nullptr;

    if (!pick.isHit) {
      return;
    }

    // resolve selected object
    auto mesh = dynamic_cast<py::SurfaceMesh*>(pick.structure);
    if (!mesh) {
      return;
    }
    auto pred = [mesh](const pIObject& obj) -> bool {
      return (obj->get_mesh() == mesh);
    };
    auto it = std::find_if(ctx_.objects.begin(), ctx_.objects.end(), pred);
    if (it == ctx_.objects.end()) {
      return;
    }
    drag_object_ = it->get();
    drag_position_ = pick.position;
    spdlog::info("Started dragging object: {}", drag_object_->get_name());
  }

  bool is_moving = (std::abs(mouse_dx) > 1 || std::abs(mouse_dy) > 1);

  // On drag: apply crtl to drag object
  if (drag_object_ && is_left_down && is_ctrl && is_moving) {
    // Hijack mouse interaction
    py::state::doDefaultMouseInteraction = false;

    // Get camera parameters for 3D conversion
    py::CameraParameters camera = py::view::getCameraParametersForCurrentView();

    float plane_dist =
        glm::dot(camera.getLookDir(), drag_position_ - camera.getPosition());

    // Calculate world space conversion factors
    float fov_y = camera.getFoVVerticalDegrees();
    float span_y = plane_dist * std::tan(3.14 * fov_y / 360.0f);
    float span_x = camera.getAspectRatioWidthOverHeight() * span_y;

    // Convert mouse delta to world space
    ImVec2 win_size = ImGui::GetMainViewport()->Size;
    float mouse_dx_ratio = mouse_dx / win_size.x;
    float mouse_dy_ratio = mouse_dy / win_size.y;

    glm::vec3 camera_right = camera.getRightDir();
    glm::vec3 camera_up = camera.getUpDir();
    glm::vec3 delta = mouse_dx_ratio * span_x * camera_right -
                      mouse_dy_ratio * span_y * camera_up;

    // Apply shift directly as glm::vec3
    drag_object_->handle_drag(delta);

    py::state::doDefaultMouseInteraction = true;
  }

  // On release: clear drag object
  if (!is_left_down && drag_object_) {
    spdlog::info("Stopped dragging object: {}", drag_object_->get_name());
    drag_object_ = nullptr;
  }
}
