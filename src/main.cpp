#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <imgui.h>
#include <spdlog/spdlog.h> // For logging

#include <iostream> 
#include <string>

#include "gui_helper.hpp"
#include "widgets/model_loader_widget.hpp"
#include "widgets/selector_widget.hpp"

namespace py = polyscope;
namespace eg = Eigen;

class Demo {
 public:
  Demo();
  void draw_imgui();
  void run();
  bool load_model_from_path(const std::string& path);

 private:
  UIContext ui_ctx_;
  EngineContext engine_ctx_;

  // Store widgets as concrete types
  ModelLoaderWidget model_loader_widget_;
  SelectorWidget selector_widget_;
  // Add other widgets here as direct members if needed
  // e.g., SomeOtherWidget some_other_widget_;
};

Demo::Demo()
    : ui_ctx_(), 
      engine_ctx_(), 
      model_loader_widget_(ui_ctx_, engine_ctx_), 
      selector_widget_(ui_ctx_) 
{
  // Set Polyscope's ImGui callback
  py::state::userCallback = [this]() { this->draw_imgui(); };
}

void Demo::draw_imgui() {
  ImGui::Begin("Controls");

  EventFlag current_event_state = NoEvent; // Initialized once per frame

  // --- Model Loader Widget ---
  model_loader_widget_.on_event(current_event_state);
  EventFlag model_loader_generated_event = model_loader_widget_.draw();
  current_event_state = static_cast<EventFlag>(
      static_cast<uint32_t>(current_event_state) |
      static_cast<uint32_t>(model_loader_generated_event));

  // --- Selector Widget ---
  selector_widget_.on_event(current_event_state);
  EventFlag selector_generated_event = selector_widget_.draw();
  current_event_state = static_cast<EventFlag>(
      static_cast<uint32_t>(current_event_state) |
      static_cast<uint32_t>(selector_generated_event));

  // --- Pattern for additional widgets ---
  // if (another_widget_) { // Or however you manage optional widgets
  //   another_widget_.on_event(current_event_state);
  //   EventFlag another_widget_generated_event = another_widget_.draw();
  //   current_event_state = static_cast<EventFlag>(
  //       static_cast<uint32_t>(current_event_state) |
  //       static_cast<uint32_t>(another_widget_generated_event));
  // }

  ImGui::End(); // End "Controls" window
}

void Demo::run() {
  py::show();
}

bool Demo::load_model_from_path(const std::string& path) {
  bool success = model_loader_widget_.load_model_from_path(path);

  if (success) {
    spdlog::info("Initial model loaded successfully from: {}", path);
    EventFlag initial_event = MeshChange;
    // Notify all relevant widgets about the mesh change
    model_loader_widget_.on_event(initial_event); 
    selector_widget_.on_event(initial_event);   
    // Call on_event for other widgets if they need to react to MeshChange
    // e.g., some_other_widget_.on_event(initial_event);
  } else {
    spdlog::error("Failed to load initial model from: {}", path);
  }
  return success;
}

int main(int argc, char** argv) {
  // Initialize Polyscope
  py::init();
  // Optional: Configure spdlog, e.g., set log level
  // spdlog::set_level(spdlog::level::info); 

  // Create the Demo application instance
  Demo demo_app;

  // Check for command-line arguments to load an initial model
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

  // Run the application
  demo_app.run();

  return 0;
}
