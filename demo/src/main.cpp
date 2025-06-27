#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <string>

#include "demo.hpp"

namespace py = polyscope;

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::debug);

  py::init();
  py::view::setUpDir(polyscope::UpDir::ZUp);
  py::view::setFrontDir(polyscope::FrontDir::XFront);
  py::options::groundPlaneMode = py::GroundPlaneMode::None;

  Demo demo_app;

  if (argc > 1) {
    std::string initial_model_path = argv[1];
    SPDLOG_INFO("Attempting to load model from command line argument: {}",
                initial_model_path);
    if (!demo_app.load_model_from_path(initial_model_path)) {
      SPDLOG_WARN(
          "Could not load initial model specified via command line: {}. "
          "Application will continue without it.",
          initial_model_path);
    }
  } else {
    SPDLOG_INFO(
        "No initial model path provided via command line arguments. "
        "Load a model using the GUI.");
  }

  demo_app.run();
  return 0;
}
