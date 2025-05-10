#include "demo.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include "gui_helper.hpp"

namespace py = polyscope;

Demo::Demo() {
  py::state::userCallback = [this]() { this->draw(); };
}

void Demo::draw() {
  EventFlag event = EventFlag::NoEvent;

  event |= mesh_stat_widget_.draw();
  event |= model_loader_widget_.draw();
  event |= selector_widget_.draw();
  event |= cloth_sim_widget_.draw();
  event |= help_bar_widget_.draw();

  model_loader_widget_.on_event(event);
  selector_widget_.on_event(event);
  cloth_sim_widget_.on_event(event);
  mesh_stat_widget_.on_event(event);
}

void Demo::run() { py::show(); }

bool Demo::load_model_from_path(const std::string& path) {
  bool success = model_loader_widget_.load_model_from_path(path);

  if (success) {
    spdlog::info("Initial model loaded successfully from: {}", path);
    model_loader_widget_.on_event(EventFlag::MeshChange);
    selector_widget_.on_event(EventFlag::MeshChange);
  } else {
    spdlog::error("Failed to load initial model from: {}", path);
  }
  return success;
}
