#include <imgui.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <spdlog/spdlog.h>

#include <string>

#include "gui_helper.hpp"
#include "widgets/model_loader_widget.hpp"
#include "widgets/selector_widget.hpp"

namespace py = polyscope;

class Demo {
  UIContext ui_ctx_ = {};
  EngineContext engine_ctx_ = {};

  ModelLoaderWidget model_loader_widget_;
  SelectorWidget selector_widget_;

  void draw_imgui();

 public:
  Demo();
  void run();
  bool load_model_from_path(const std::string& path);
};

Demo::Demo()
    : ui_ctx_(),
      engine_ctx_(),
      model_loader_widget_(ui_ctx_, engine_ctx_),
      selector_widget_(ui_ctx_) {
  py::state::userCallback = [this]() { this->draw_imgui(); };
}

void Demo::draw_imgui() {
  EventFlag event = EventFlag::NoEvent;

  event |= model_loader_widget_.draw();
  event |= selector_widget_.draw();

  model_loader_widget_.on_event(event);
  selector_widget_.on_event(event);
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

int main(int argc, char** argv) {
  py::init();
  Demo demo_app;

  if (argc > 1) {
    std::string initial_model_path = argv[1];
    spdlog::info("Attempting to load model from command line argument: {}",
                 initial_model_path);
    if (!demo_app.load_model_from_path(initial_model_path)) {
      spdlog::warn(
          "Could not load initial model specified via command line: {}. "
          "Application will continue without it.",
          initial_model_path);
    }
  } else {
    spdlog::info(
        "No initial model path provided via command line arguments. "
        "Load a model using the GUI.");
  }

  demo_app.run();
  return 0;
}
