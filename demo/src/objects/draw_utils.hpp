#pragma once

#include <glm/glm.hpp>

#include "silk/silk.hpp"

void draw_cloth_config(silk::ClothConfig& cloth_config,
                       bool& cloth_config_changed);

void draw_collision_config(silk::CollisionConfig& collision_config,
                           bool& collision_config_changed);

void draw_transform_widget(glm::vec3& position, glm::vec3& rotation,
                           float& scale, bool& transform_changed);
