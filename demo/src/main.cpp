#include <imgui.h>
#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <optional>
#include <string>

#include "config.hpp"
#include "demo.hpp"
#include "json_parse.hpp"

namespace py = polyscope;

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::debug);

  argparse::ArgumentParser program("Silk Demo");
  program.add_description("A Blazingly fast projective dynamics cloth solver");

  std::string config_path;
  program.add_argument("-c", "--config")
      .help("The all in one simulation config file")
      .store_into(config_path);

  bool is_headless = false;
  program.add_argument("-h", "--headless")
      .help("Run the simulation in headless cli")
      .flag()
      .store_into(is_headless);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    spdlog::error("Fail to parse cli args. Reason: {}", err.what());
    return 1;
  }

  std::optional<SimConfig> sim_config = std::nullopt;
  if (!config_path.empty()) {
    // TODO: call json parsing routine here.
    sim_config = parse_config(config_path);
    if (!sim_config) {
      spdlog::error("Fail to parse config file {}.", config_path);
    } else {
      spdlog::info("Load config file {}. Currently doing nothing", config_path);
    }
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
  if (sim_config) {
    demo_app.apply_config(*sim_config);
  }

  demo_app.run();

  return 0;
}
