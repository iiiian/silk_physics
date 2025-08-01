#include "demo.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include "gui_helper.hpp"

namespace py = polyscope;

Demo::Demo() {
  py::state::userCallback = [this]() { this->draw(); };
}

void Demo::draw() {
  ImGui::Text("FPS %f", ImGui::GetIO().Framerate);
  ImGui::Separator();
  ImGui::DragFloat("UI Scale", &ImGui::GetIO().FontGlobalScale, 0.1f, 1.0f,
                   3.0f);

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
    SPDLOG_INFO("Initial model loaded successfully from: {}", path);
    model_loader_widget_.on_event(EventFlag::MeshChange);
    selector_widget_.on_event(EventFlag::MeshChange);
  } else {
    SPDLOG_ERROR("Failed to load initial model from: {}", path);
  }
  return success;
}
