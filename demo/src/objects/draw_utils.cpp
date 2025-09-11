#include "draw_utils.hpp"

#include <polyscope/polyscope.h>

void draw_cloth_config(silk::ClothConfig& cloth_config,
                       bool& cloth_config_changed) {
  ImGui::SeparatorText("Cloth Setting");

  if (ImGui::InputFloat("Elastic Stiffness", &cloth_config.elastic_stiffness)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Bending Stiffness", &cloth_config.bending_stiffness)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Density", &cloth_config.density)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Damping", &cloth_config.damping)) {
    cloth_config_changed = true;
  }
}

void draw_collision_config(silk::CollisionConfig& collision_config,
                           bool& collision_config_changed) {
  ImGui::SeparatorText("Collision Setting");

  if (ImGui::Checkbox("Is Collision On", &collision_config.is_collision_on)) {
    collision_config_changed = true;
  }
  if (ImGui::Checkbox("Is Self Collision On",
                      &collision_config.is_self_collision_on)) {
    collision_config_changed = true;
  }
  if (ImGui::InputInt("Collision Group", &collision_config.group)) {
    collision_config_changed = true;
  }
  if (ImGui::InputFloat("Restitution", &collision_config.restitution)) {
    collision_config_changed = true;
  }
  if (ImGui::InputFloat("Friction", &collision_config.friction)) {
    collision_config_changed = true;
  }
}

void draw_transform_widget(glm::vec3& position, glm::vec3& rotation,
                           float& scale, bool& transform_changed) {
  ImGui::SeparatorText("Transform");

  if (ImGui::DragFloat3("Position", &position.x, 0.01f, -100.0f, 100.0f,
                        "%.3f")) {
    transform_changed = true;
  }

  glm::vec3 rotation_degrees = glm::degrees(rotation);
  if (ImGui::DragFloat3("Rotation", &rotation_degrees.x, 1.0f, -180.0f, 180.0f,
                        "%.1f°")) {
    rotation = glm::radians(rotation_degrees);
    transform_changed = true;
  }

  if (ImGui::DragFloat("Scale", &scale, 0.01f, 0.01f, 10.0f, "%.3f")) {
    transform_changed = true;
  }

  if (ImGui::Button("Reset Transform")) {
    position = glm::vec3(0.0f);
    rotation = glm::vec3(0.0f);
    scale = 1.0f;
    transform_changed = true;
  }
}
