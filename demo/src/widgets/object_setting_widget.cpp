#include "object_setting_widget.hpp"

#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <glm/glm.hpp>

#include "../gui_helper.hpp"

namespace py = polyscope;

void ObjectSettingWidget::enter_paint_mode() {
  Object& obj = ctx_.objects[ctx_.selection];

  py::state::doDefaultMouseInteraction = false;
  obj.mesh->setSelectionMode(py::MeshSelectionMode::Auto);

  selector_sphere_ = py::registerPointCloud(
      "selector_sphere", std::vector<glm::vec3>{selector_center_});
  selector_sphere_->setPointRadius(selector_radius_, false);
  selector_sphere_->setPointColor({1.0f, 0.5f, 0.0f});
  selector_sphere_->setTransparency(0.5f);

  kd_tree_ = std::make_unique<KDTree>(obj.V);

  ctx_.ui_mode = UIMode::Paint;
  SPDLOG_INFO("Entered paint mode.");
}

void ObjectSettingWidget::leave_paint_mode() {
  py::state::doDefaultMouseInteraction = true;
  py::removeStructure("selector_sphere");
  selector_sphere_ = nullptr;

  kd_tree_ = {};

  ctx_.ui_mode = UIMode::Normal;
  SPDLOG_INFO("Left paint mode.");
}

void ObjectSettingWidget::update_selection_visual() {
  Object& obj = ctx_.objects[ctx_.selection];

  std::vector<double> indicator(obj.mesh->nVertices(), 0.0);
  for (int idx : obj.pinned) {
    indicator[idx] = 1.0;
  }
  obj.mesh->addVertexScalarQuantity("pinned", indicator)->setEnabled(true);
}

void ObjectSettingWidget::select_vertices_in_sphere(bool add_to_selection) {
  Eigen::Vector3f center(selector_center_.x, selector_center_.y,
                         selector_center_.z);
  std::vector<int> matches = kd_tree_->find_neighbors(center, selector_radius_);

  Object& obj = ctx_.objects[ctx_.selection];
  size_t changed_count = 0;
  if (add_to_selection) {
    for (auto match : matches) {
      if (obj.pinned.insert(match).second) {
        changed_count++;
      }
    }
  } else {
    for (auto match : matches) {
      if (obj.pinned.erase(match) > 0) {
        changed_count++;
      }
    }
  }

  if (changed_count > 0) {
    update_selection_visual();
    obj.pinned_changed = true;
  }
}

void ObjectSettingWidget::handle_paint_input() {
  if (ctx_.ui_mode != UIMode::Paint) {
    return;
  }

  Object& obj = ctx_.objects[ctx_.selection];

  ImVec2 mouse_pos = ImGui::GetMousePos();
  float delta_x = std::abs(mouse_pos.x - prev_mouse_pos_.x);
  float delta_y = std::abs(mouse_pos.y - prev_mouse_pos_.y);
  if (delta_x > 1 || delta_y > 1) {
    prev_mouse_pos_ = mouse_pos;
    py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});

    is_mouse_on_surface_ = pick.isHit;
    if (pick.isHit && pick.structure == obj.mesh) {
      selector_center_ = pick.position;
      selector_sphere_->updatePointPositions(
          std::vector<glm::vec3>{selector_center_});
    }
  }

  if (!is_mouse_on_surface_) {
    return;
  }

  bool left_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool right_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);
  if (left_mouse_down) {
    select_vertices_in_sphere(true);
  } else if (right_mouse_down) {
    select_vertices_in_sphere(false);
  }
}

ObjectSettingWidget::ObjectSettingWidget(Context& context) : ctx_(context) {}

void ObjectSettingWidget::draw() {
  ImGui::BeginDisabled((ctx_.selection == -1));

  if (ImGui::CollapsingHeader("Object Settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ctx_.selection != -1) {
      Object& obj = ctx_.objects[ctx_.selection];

      // object type combo box
      ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
      const char* type_names[] = {"None", "Cloth", "Obstacle"};
      int type_idx = static_cast<int>(obj.type);
      if (ImGui::Combo("Object Type", &type_idx, type_names,
                       IM_ARRAYSIZE(type_names))) {
        obj.type = static_cast<SilkObjectType>(type_idx);
      }
      ImGui::EndDisabled();

      // cloth related settings
      if (obj.type == SilkObjectType::Cloth) {
        ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
        draw_cloth_setting();
        draw_collision_setting();
        ImGui::EndDisabled();

        ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal &&
                             ctx_.ui_mode != UIMode::Paint);
        draw_pin_group_setting();
        ImGui::EndDisabled();
      }

      // obstacle related settings
      if (obj.type == SilkObjectType::Obstacle) {
        ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
        draw_collision_setting();
        ImGui::EndDisabled();
      }
    }
  }

  ImGui::EndDisabled();
}

void ObjectSettingWidget::draw_type_combo() {
  Object& obj = ctx_.objects[ctx_.selection];

  const char* type_names[] = {"None", "Cloth", "Obstacle"};
  int type_idx = static_cast<int>(obj.type);
  if (ImGui::Combo("Object Type", &type_idx, type_names,
                   IM_ARRAYSIZE(type_names))) {
    // remove silk object
    if (obj.silk_handle != 0) {
      switch (obj.type) {
        case SilkObjectType::None:
          assert(false);
          break;
        case SilkObjectType::Cloth: {
          auto res = ctx_.silk_world.remove_cloth(obj.silk_handle);
          assert((res == silk::Result::Success));
          break;
        }
        case SilkObjectType::Obstacle: {
          auto res = ctx_.silk_world.remove_obstacle(obj.silk_handle);
          assert((res == silk::Result::Success));
          break;
        }
      }
      obj.silk_handle = 0;
    }

    obj.type = static_cast<SilkObjectType>(type_idx);
  }
}

void ObjectSettingWidget::draw_cloth_setting() {
  ImGui::SeparatorText("Cloth Setting");

  Object& obj = ctx_.objects[ctx_.selection];
  silk::ClothConfig& c = obj.cloth_config;

  if (ImGui::InputFloat("Elastic Stiffness", &c.elastic_stiffness)) {
    obj.physical_config_changed = true;
  }

  if (ImGui::InputFloat("Bending Stiffness", &c.bending_stiffness)) {
    obj.physical_config_changed = true;
  }

  if (ImGui::InputFloat("Density", &c.density)) {
    obj.physical_config_changed = true;
  }
}

void ObjectSettingWidget::draw_collision_setting() {
  ImGui::SeparatorText("Collision Setting");

  Object& obj = ctx_.objects[ctx_.selection];
  silk::CollisionConfig& c = obj.collision_config;

  if (ImGui::Checkbox("Is Collision On", &c.is_collision_on)) {
    obj.collision_config_changed = true;
  }

  if (ImGui::Checkbox("Is Self Collision On", &c.is_self_collision_on)) {
    obj.collision_config_changed = true;
  }

  if (ImGui::InputInt("Collision Group", &c.group)) {
    obj.collision_config_changed = true;
  }

  if (ImGui::InputFloat("Damping", &c.damping)) {
    obj.collision_config_changed = true;
  }

  if (ImGui::InputFloat("Friction", &c.friction)) {
    obj.collision_config_changed = true;
  }
}

void ObjectSettingWidget::draw_pin_group_setting() {
  ImGui::SeparatorText("Collision Setting");

  Object& obj = ctx_.objects[ctx_.selection];

  ImGui::Text("%d vertices pinned", int(obj.pinned.size()));

  ImGui::DragFloat("Brush Size", &selector_radius_);

  bool is_painting = (ctx_.ui_mode == UIMode::Paint);
  if (ImGui::Button(is_painting ? "Paint Pinned Vertices" : "Stop Painting")) {
    if (is_painting) {
      leave_paint_mode();
    } else {
      enter_paint_mode();
    }
  }

  if (is_painting) {
    handle_paint_input();
  }
}
