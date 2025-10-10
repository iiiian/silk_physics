#include "demo.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

namespace py = polyscope;

Demo::Demo() {
  py::state::userCallback = [this]() { this->draw(); };
}

void Demo::draw() {
  // left panel
  const auto& display_size = ImGui::GetIO().DisplaySize;
  const float panel_width = 400.f * py::options::uiScale;
  ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
  ImGui::SetNextWindowSize(ImVec2(panel_width, display_size.y));
  ImGui::Begin("Left Panel", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoCollapse);
  statistic_widget_.draw();
  scene_widget_.draw();
  object_setting_widget_.draw();
  ImGui::End();

  // right panel
  ImGui::SetNextWindowPos(ImVec2(display_size.x - panel_width, 0.f));
  ImGui::SetNextWindowSize(ImVec2(panel_width, display_size.y));
  ImGui::Begin("Right Panel", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoCollapse);
  sim_setting_widget_.draw();
  simulator_widget_.draw();
    //*******************************/
  gpu_solver_widget_.draw();
  //*******************************/
  ImGui::End();

  help_bar_widget_.draw();
}

void Demo::run() { py::show(); }

bool Demo::load_model_from_path(const std::string& path) {
  // Default initial load as a cloth object
  if (scene_widget_.load_object_from_path(path, SilkObjectType::Cloth)) {
    spdlog::info("Initial model loaded successfully from: {}", path);
    return true;
  }

  spdlog::error("Failed to load initial model from: {}", path);
  return false;
}
