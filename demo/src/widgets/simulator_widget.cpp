#include "simulator_widget.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <vector>

namespace py = polyscope;

void SimulatorWidget::init_cloth(Object& obj) {
  bool is_changed = obj.physical_config_changed ||
                    obj.collision_config_changed || obj.pinned_changed;

  if (is_changed && obj.silk_handle != 0) {
    auto res = ctx_.silk_world.remove_cloth(obj.silk_handle);
    assert((res == silk::Result::Success));
  }

  if (obj.silk_handle == 0) {
    std::vector<int> pinned;
    pinned.insert(pinned.end(), obj.pinned.begin(), obj.pinned.end());
    silk::ConstSpan<int> pinnd_span{pinned.data(), int(pinned.size())};

    std::vector<float> vert_data;
    obj.mesh->vertexPositions.ensureHostBufferPopulated();
    assert(obj.mesh->vertexPositions.data.size() != 0);
    for (auto vec3 : obj.mesh->vertexPositions.data) {
      vert_data.push_back(vec3[0]);
      vert_data.push_back(vec3[1]);
      vert_data.push_back(vec3[2]);
    }

    silk::MeshConfig mesh_config;
    mesh_config.verts = {vert_data.data(), int(vert_data.size())};
    mesh_config.faces = {obj.F.data(), int(obj.F.size())};

    auto res =
        ctx_.silk_world.add_cloth(obj.cloth_config, obj.collision_config,
                                  mesh_config, pinnd_span, obj.silk_handle);
    assert((res == silk::Result::Success));
  }
}

void SimulatorWidget::init_obstacle(Object& obj) {
  bool is_changed = obj.physical_config_changed ||
                    obj.collision_config_changed || obj.pinned_changed;

  if (is_changed) {
    auto res = ctx_.silk_world.remove_obstacle(obj.silk_handle);
    assert((res == silk::Result::Success));
  }

  if (obj.silk_handle == 0) {
    std::vector<float> vert_data;
    obj.mesh->vertexPositions.ensureHostBufferPopulated();
    for (auto vec3 : obj.mesh->vertexPositions.data) {
      vert_data.push_back(vec3[0]);
      vert_data.push_back(vec3[1]);
      vert_data.push_back(vec3[2]);
    }

    silk::MeshConfig mesh_config;
    mesh_config.verts = {vert_data.data(), int(vert_data.size())};

    auto res = ctx_.silk_world.add_obstacle(obj.collision_config, mesh_config,
                                            obj.silk_handle);
    assert((res == silk::Result::Success));
  }
}

void SimulatorWidget::enter_sim_mode() {
  for (auto& obj : ctx_.objects) {
    switch (obj.type) {
      case SilkObjectType::None:
        break;
      case SilkObjectType::Cloth:
        init_cloth(obj);
        break;
      case SilkObjectType::Obstacle:
        init_obstacle(obj);
        break;
    }
  }

  auto res = ctx_.silk_world.set_global_config(ctx_.global_config);
  assert(res == silk::Result::Success);
  res = ctx_.silk_world.solver_init();
  assert(res == silk::Result::Success);

  py::state::doDefaultMouseInteraction = false;
  is_first_click = true;
  selected_obj = nullptr;
  ctx_.ui_mode = UIMode::Sim;
  SPDLOG_INFO("Enter sim mode");
}

void SimulatorWidget::leave_sim_mode() {
  for (auto& obj : ctx_.objects) {
    obj.mesh->updateVertexPositions(obj.V);
  }

  py::state::doDefaultMouseInteraction = true;
  ctx_.ui_mode = UIMode::Normal;
  SPDLOG_INFO("Leave cloth sim mode");
}

void SimulatorWidget::update_pin(const Object& obj) {
  if (obj.type != SilkObjectType::Cloth || obj.pinned.empty()) {
    return;
  }

  obj.mesh->vertexPositions.ensureHostBufferPopulated();
  auto& vpos = obj.mesh->vertexPositions.data;
  std::vector<float> pin_pos;
  for (int i : obj.pinned) {
    pin_pos.push_back(vpos[i][0]);
    pin_pos.push_back(vpos[i][1]);
    pin_pos.push_back(vpos[i][2]);
  }
  auto res = ctx_.silk_world.set_cloth_pin_position(
      obj.silk_handle, {pin_pos.data(), int(pin_pos.size())});
  assert((res == silk::Result::Success));
}

void SimulatorWidget::update_pos(Object& obj) {
  if (obj.type != SilkObjectType::Cloth) {
    return;
  }

  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> V;
  V.resize(obj.V.rows(), 3);
  auto res = ctx_.silk_world.get_cloth_position(obj.silk_handle,
                                                {V.data(), int(V.size())});
  assert((res == silk::Result::Success));

  obj.mesh->updateVertexPositions(V);
}

void SimulatorWidget::solver_step() {
  for (auto& obj : ctx_.objects) {
    update_pin(obj);
  }

  auto res = ctx_.silk_world.solver_step();
  if (res != silk::Result::Success) {
    SPDLOG_ERROR("solve fail, Reason: {}", silk::to_string(res));
    leave_sim_mode();
    return;
  }

  for (Object& obj : ctx_.objects) {
    update_pos(obj);
  }
}

void SimulatorWidget::handle_drag_selection() {
  bool is_left_click = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool is_ctrl = ImGui::GetIO().KeyCtrl;

  if (!is_left_click || !is_ctrl) {
    is_first_click = true;
    selected_obj = nullptr;
    py::state::doDefaultMouseInteraction = true;
    return;
  }

  ImVec2 mouse_pos = ImGui::GetMousePos();
  if (is_first_click) {
    py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});
    if (pick.isHit) {
      auto pred = [structure = pick.structure](Object& obj) -> bool {
        return (obj.mesh == structure);
      };
      auto it = std::find_if(ctx_.objects.begin(), ctx_.objects.end(), pred);
      if (it != ctx_.objects.end() && !it->pinned.empty()) {
        selected_obj = &(*it);
      }
    }
  }

  if (!selected_obj || (mouse_pos[0] == prev_mouse_pos_[0] &&
                        mouse_pos[1] == prev_mouse_pos_[1])) {
    return;
  }

  // hijack mouse handling
  py::state::doDefaultMouseInteraction = false;

  py::CameraParameters camera = py::view::getCameraParametersForCurrentView();
  // choose a random selected vertex to calculate distance
  glm::vec3 rand_pos =
      selected_obj->mesh->vertexPositions.data[*selected_obj->pinned.begin()];
  float plane_dist =
      glm::dot(camera.getLookDir(), rand_pos - camera.getPosition());

  float fov_y = camera.getFoVVerticalDegrees();
  float span_y = plane_dist * std::tan(3.14f * fov_y / 360);
  float span_x = camera.getAspectRatioWidthOverHeight() * span_y;

  ImVec2 win_size = ImGui::GetMainViewport()->Size;
  float mouse_dx = mouse_pos.x - prev_mouse_pos_.x;
  float mouse_dy = mouse_pos.y - prev_mouse_pos_.y;
  float mouse_dx_ratio = mouse_dx / win_size.x;
  float mouse_dy_ratio = mouse_dy / win_size.y;

  glm::vec3 camera_right = camera.getRightDir();
  glm::vec3 camera_up = camera.getUpDir();
  glm::vec3 delta = mouse_dx_ratio * span_x * camera_right -
                    mouse_dy_ratio * span_y * camera_up;

  selected_obj->mesh->vertexPositions.ensureHostBufferPopulated();
  auto& vpos = selected_obj->mesh->vertexPositions.data;
  for (int i : selected_obj->pinned) {
    vpos[i] += delta;
  }
  selected_obj->mesh->vertexPositions.markHostBufferUpdated();
}

SimulatorWidget::SimulatorWidget(Context& context) : ctx_(context) {}

void SimulatorWidget::draw() {
  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
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
  }

  if (ctx_.ui_mode == UIMode::Sim) {
    handle_drag_selection();
    prev_mouse_pos_ = ImGui::GetMousePos();

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapse_sec = now - prev_update_time_;
    if (elapse_sec.count() >= ctx_.global_config.dt) {
      solver_step();
      prev_update_time_ = std::chrono::steady_clock::now();
    }
  }
}
