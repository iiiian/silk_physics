#include "config_widget.hpp"

#include <imgui.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <portable-file-dialogs.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <filesystem>

#include "../json_parse.hpp"
#include "ui_console.hpp"

// default path
static const char* DefaultJsonPath = "/DTest/default.json";

// helper
static bool is_json_path(const std::string& path) {
  std::filesystem::path p{path};
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".json";
}

ConfigWidget::ConfigWidget(Context& ctx, std::function<void(jsonPath)> status)
    : ctx_(ctx), status_(std::move(status)) {}
// =====================================================

void ConfigWidget::set_path(jsonPath p) { path = p; }

void ConfigWidget::draw() {
  ImGui::PushID(this);

  if (ImGui::CollapsingHeader((title_ + "##config").c_str(),
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    bool useDefault = (path == jsonPath::DEFAULT);
    if (ImGui::Checkbox("Use Default Json File##config_checkbox",
                        &useDefault)) {
      path = useDefault ? jsonPath::DEFAULT : jsonPath::CUSTOM;
      UI_LOGI("[UI] Json File -> {}", useDefault ? "Default" : "Custom");
    }

    // Choose Custom JSON File
    ImGui::BeginDisabled(useDefault);
    if (ImGui::Button("Load JSON##config_btn")) {
      const std::vector<std::string> filters = {"JSON File", "*.json"};
      std::vector<std::string> results =
          pfd::open_file("Select JSON config", ".", filters, pfd::opt::none)
              .result();

      if (!results.empty()) {
        const std::string& picked = results[0];

        if (!is_json_path(picked)) {
          UI_LOGE("[Config] Invalid file type. Please select a .json file: {}",
                  picked);
          pfd::message("Invalid file", "Please select a .json file.",
                       pfd::choice::ok, pfd::icon::error);
        } else {
          picked_path_ = picked;

          std::filesystem::path sel{picked_path_};
          UI_LOGI("[Config] Selected JSON file name: {}",
                  sel.filename().string());

          // TEST (readable)
          ParseResult r = parse_config(picked_path_, DefaultJsonPath, true);
          if (r.ok) {
            UI_LOGI("JSON Parsing Sucessed: {}", r.source_path);

          } else {
            UI_LOGE("JSON Parsing Failed:{}", r.error);
          }
        }
      } else {
        UI_LOGW("[Config] No JSON file selected.");
      }
    }
    ImGui::EndDisabled();
  }

  ImGui::PopID();
}
