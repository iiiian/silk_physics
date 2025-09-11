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