#include "object_setting_widget.hpp"

#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <glm/glm.hpp>

#include "../gui_utils.hpp"
#include "../object.hpp"

namespace py = polyscope;

void ObjectSettingWidget::enter_paint_mode() {
  py::state::doDefaultMouseInteraction = false;

  if (!selector_sphere_) {
    selector_sphere_ = py::registerPointCloud(
        "selector_sphere", std::vector<glm::vec3>{selector_center_});
    selector_sphere_->setPointRadius(selector_radius_, false);
    selector_sphere_->setPointColor({1.0f, 0.5f, 0.0f});
    selector_sphere_->setTransparency(0.5f);
  }

  ctx_.ui_mode = UIMode::Paint;
  spdlog::info("Entered paint mode.");
}

void ObjectSettingWidget::leave_paint_mode() {
  py::state::doDefaultMouseInteraction = true;
  py::removeStructure("selector_sphere");
  selector_sphere_ = nullptr;

  ctx_.ui_mode = UIMode::Normal;
  spdlog::info("Left paint mode.");
}

void ObjectSettingWidget::handle_paint_input() {
  ImVec2 mouse_pos = ImGui::GetMousePos();
  float delta_x = std::abs(mouse_pos.x - prev_mouse_pos_.x);
  float delta_y = std::abs(mouse_pos.y - prev_mouse_pos_.y);
  bool left_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool right_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);
  if (delta_x < 1 && delta_y < 1 && !left_mouse_down && !right_mouse_down) {
    return;
  }

  prev_mouse_pos_ = mouse_pos;

  // disable selector sphere during pick so it wont pick itself
  selector_sphere_->setEnabled(false);
  py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});
  selector_sphere_->setEnabled(true);

  if (!pick.isHit) {
    return;
  }

  // update selector sphere position
  selector_center_ = pick.position;
  std::vector<glm::vec3> center = {selector_center_};
  selector_sphere_->updatePointPositions(center);

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
  auto& object = *it;

  // scaled selector sphere based on object scale
  selector_radius_ = 0.05f * object->get_object_scale();
  selector_sphere_->setPointRadius(selector_radius_, false);

  // pass pick result so object can update pin selection
  if (left_mouse_down) {
    object->handle_pick(pick, true, pick_radius_);
  } else if (right_mouse_down) {
    object->handle_pick(pick, false, pick_radius_);
  }
}

ObjectSettingWidget::ObjectSettingWidget(Context& context) : ctx_(context) {}

void ObjectSettingWidget::draw() {
  ImGui::BeginDisabled((ctx_.selection == -1));

  if (ImGui::CollapsingHeader("Object Settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ctx_.selection != -1) {
      auto& obj = ctx_.objects[ctx_.selection];

      // Object specific settings like elastic stiffness etc.
      ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
      obj->draw();
      ImGui::EndDisabled();

      ImGui::SeparatorText("Paint");

      // Paint mode controls.
      ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal &&
                           ctx_.ui_mode != UIMode::Paint);
      bool is_painting = (ctx_.ui_mode == UIMode::Paint);
      // painting mode button
      if (ImGui::Button(is_painting ? "Stop Painting" : "Start Painting")) {
        if (is_painting) {
          leave_paint_mode();
        } else {
          enter_paint_mode();
        }
      }
      is_painting = (ctx_.ui_mode == UIMode::Paint);

      // painting mode pick radius
      ImGui::SliderInt("Paint Radius", &pick_radius_, 0, 30);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Breadth-first depth in mesh adjacency. 0 = single vertex.");
      }
      ImGui::EndDisabled();

      if (is_painting) {
        handle_paint_input();
      }
    }
  }

  ImGui::EndDisabled();
}

// removed legacy type/config/pin-group UI; now delegated to IObject
