#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>
#include <argparse/argparse.hpp>
#include <string>
#include "demo.hpp"
#include "config_parser.hpp"

namespace py = polyscope;

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::debug);

  argparse::ArgumentParser program("Silk Demo");
  program.add_description("A Blazing fast projective dynamics cloth solver");

  std::string config_path;
  program.add_argument("-c", "--config").help("Simulation config file").store_into(config_path);

  bool is_headless = false;
  program.add_argument("-H", "--headless").help("Run in headless mode").flag().store_into(is_headless);

  std::string cloth_model_path;
  program.add_argument("cloth_model_path")
      .help("Optional model path (deprecated; prefer -c)")
      .nargs(0, 1)
      .store_into(cloth_model_path);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    spdlog::error("Fail to parse cli args. Reason: {}", err.what());
    return 1;
  }

  cli::SimConfig simcfg;
  std::string perr;
  bool has_config = false;

  if (!config_path.empty()) {
    if (!cli::load_sim_config(config_path, simcfg, perr)) {
      spdlog::error("Config error: {}", perr);
      return 1;
    }
    has_config = true;
    spdlog::info("Config loaded from {}", config_path);
    spdlog::info("  dt = {}, steps = {}, headless = {}", simcfg.global.dt, simcfg.stop.total_steps, simcfg.headless);
    if (cloth_model_path.empty() && !simcfg.objects.empty()) {
      cloth_model_path = simcfg.objects.front().mesh_path;
    }
    if (!is_headless) {
      is_headless = simcfg.headless;
    }
  }

  if (is_headless) {
    spdlog::info("Headless mode");
    if (has_config) {
      spdlog::info("  objects = {}", simcfg.objects.size());
      spdlog::info("  dt = {}, steps = {}", simcfg.global.dt, simcfg.stop.total_steps);
    }
    return 0;
  }

  py::options::buildGui = false;
  py::init();
  py::view::setUpDir(polyscope::UpDir::ZUp);
  py::view::setFrontDir(polyscope::FrontDir::XFront);
  py::options::groundPlaneMode = py::GroundPlaneMode::None;

  Demo demo_app;

  if (!cloth_model_path.empty()) {
    spdlog::info("Attempting to load model: {}", cloth_model_path);
    if (!demo_app.load_model_from_path(cloth_model_path)) {
      spdlog::warn("Could not load initial model: {}", cloth_model_path);
    } else {
      spdlog::info("Successfully loaded initial model: {}", cloth_model_path);
    }
  } else {
    spdlog::info("No initial model path provided. Load a model using the GUI.");
  }

  demo_app.run();
  return 0;
}
