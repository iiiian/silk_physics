#include "config_widget.hpp"

#include <imgui.h>
#include <portable-file-dialogs.h>

#include <filesystem>

#include "../json_parse.hpp"
#include "../objects/cloth.hpp"
#include "../objects/obstacle.hpp"
#include "ui_console.hpp"

ConfigWidget::ConfigWidget(Context& ctx) : ctx_(ctx) {}

void ConfigWidget::draw() {
  ImGui::PushID(this);

  if (ImGui::CollapsingHeader("Config Selection##config",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    // Choose Custom JSON File
    if (ImGui::Button("Load JSON##config_btn")) {
      const std::vector<std::string> filters = {"JSON File", "*.json"};
      std::vector<std::string> results =
          pfd::open_file("Select JSON config", ".", filters, pfd::opt::none)
              .result();

      if (!results.empty()) {
        std::filesystem::path picked{results[0]};

        if (picked.extension() != ".json") {
          ui_error("[Config] Invalid file type. Please select a .json file.");
          pfd::message("Invalid file", "Please select a .json file.",
                       pfd::choice::ok, pfd::icon::error);
        } else {
          ui_info("[Config] Selected JSON file name: {}", picked);

          auto cfg = parse_config(picked);
          if (cfg) {
            ui_info("JSON {} Parsing Sucessed.", picked);
            apply_config_to_gui(*cfg);
          } else {
            ui_error("JSON {} Parsing Failed.", picked);
          }
        }
      }
    }
  }

  ImGui::PopID();
}

void ConfigWidget::apply_config_to_gui(const SimConfig& config) {
  auto& c = config;
  // Modify current global config
  ctx_.global_config.dt = c.global.dt;
  ctx_.global_config.max_inner_iteration = c.global.max_inner_iteration;
  ctx_.global_config.max_outer_iteration = c.global.max_outer_iteration;
  ctx_.global_config.acceleration_x = c.global.acceleration[0];
  ctx_.global_config.acceleration_y = c.global.acceleration[1];
  ctx_.global_config.acceleration_z = c.global.acceleration[2];

  for (auto& cloth_cfg : c.cloths) {
    auto cloth = Cloth::make_cloth(&ctx_.silk_world, cloth_cfg);
    if (!cloth) {
      spdlog::error("Fail to load cloth from config.");
      continue;
    }
    auto ptr = std::make_unique<Cloth>(std::move(*cloth));
    ctx_.objects.push_back(std::move(ptr));
  }

  for (auto& obstacle_cfg : c.obstacles) {
    auto obstacle = Obstacle::make_obstacle(&ctx_.silk_world, obstacle_cfg);
    if (!obstacle) {
      spdlog::error("Fail to load obstacle from config.");
      continue;
    }
    auto ptr = std::make_unique<Obstacle>(std::move(*obstacle));
    ctx_.objects.push_back(std::move(ptr));
  }
}
