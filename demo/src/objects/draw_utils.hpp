#pragma once

#include "silk/silk.hpp"

void draw_cloth_config(silk::ClothConfig& cloth_config,
                       bool& cloth_config_changed);
void draw_collision_config(silk::CollisionConfig& collision_config,
                           bool& collision_config_changed);