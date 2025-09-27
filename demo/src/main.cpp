#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <string>

#include "demo.hpp"

namespace py = polyscope;

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::debug);

  argparse::ArgumentParser program("Silk Demo");
  program.add_description("A Blazing fast projective dynamics cloth solver");

  std::string config_path;
  program.add_argument("-c", "--config")
      .help("The all in one simulation config file")
      .store_into(config_path);

  bool is_headless = false;
  program.add_argument("-h", "--headless")
      .help("Run the simulation in headless cli")
      .flag()
      .store_into(is_headless);

  std::string cloth_model_path;
  program.add_argument("cloth_model_path")
      .help("Path to cloth model file. Soon to be deprecated")
      .store_into(cloth_model_path);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    spdlog::error("Fail to parse cli args. Reason: {}", err.what());
    return 1;
  }

  if (!config_path.empty()) {
    // TODO: call json parsing routine here.
    spdlog::info("Load config file {}. Currently doing nothing", config_path);
  }

  if (is_headless) {
    // TODO: lauch cli mode here.
    spdlog::info("Headless mode. Currently doing nothing");
    return 0;
  }

  py::options::buildGui = false;
  py::init();
  py::view::setUpDir(polyscope::UpDir::ZUp);
  py::view::setFrontDir(polyscope::FrontDir::XFront);
  py::options::groundPlaneMode = py::GroundPlaneMode::None;

  Demo demo_app;

  if (!cloth_model_path.empty()) {
    spdlog::info("Attempting to load model from command line argument: {}",
                 cloth_model_path);
    if (!demo_app.load_model_from_path(cloth_model_path)) {
      spdlog::warn(
          "Could not load initial model specified via command line: {}. "
          "Application will continue without it.",
          cloth_model_path);
    } else {
      spdlog::info(
          "No initial model path provided via command line arguments. "
          "Load a model using the GUI.");
    }
  }

  demo_app.run();

  return 0;
}
