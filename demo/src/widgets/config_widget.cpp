#include "config_widget.hpp"

#include <imgui.h>
#include <portable-file-dialogs.h>

#include <filesystem>

#include "../json_parse.hpp"
#include "ui_console.hpp"

ConfigWidget::ConfigWidget(Context& ctx) : ctx_(ctx) {}

void ConfigWidget::draw() {
  ImGui::PushID(this);

  if (ImGui::CollapsingHeader((title_ + "##config").c_str(),
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Use Default Json File##config_checkbox",
                        &use_default_json_)) {
      ui_info("[UI] Json File -> {}", use_default_json_ ? "Default" : "Custom");
    }

    // Choose Custom JSON File
    ImGui::BeginDisabled(use_default_json_);
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
          } else {
            ui_error("JSON {} Parsing Failed.", picked);
          }
        }
      } else {
        ui_warning("[Config] No JSON file selected.");
      }
    }
    ImGui::EndDisabled();
  }

  ImGui::PopID();
}
