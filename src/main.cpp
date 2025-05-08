#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <string>

#include "demo.hpp"

namespace py = polyscope;

int main(int argc, char** argv) {
  py::init();
  py::view::setUpDir(polyscope::UpDir::ZUp);
  py::options::groundPlaneMode = py::GroundPlaneMode::None;
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
