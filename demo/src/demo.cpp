#include "demo.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

namespace py = polyscope;

Demo::Demo() {
  py::state::userCallback = [this]() { this->draw(); };
}

void Demo::draw() {
  // left panel
  statistic_widget_.draw();
  scene_widget_.draw();
  object_setting_widget_.draw();

  // right panel
  sim_setting_widget_.draw();
  simulator_widget_.draw();
}

void Demo::run() { py::show(); }

bool Demo::load_model_from_path(const std::string& path) {
  bool success = scene_widget_.load_object_from_path(path);

  if (success) {
    SPDLOG_INFO("Initial model loaded successfully from: {}", path);
  } else {
    SPDLOG_ERROR("Failed to load initial model from: {}", path);
  }
  return success;
}
