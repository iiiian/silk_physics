#include <imgui.h>
#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <optional>
#include <cctype>
#include <string>

#include "config.hpp"
#include "demo.hpp"
#include "headless.hpp"
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

  std::string out_path = "out.abc";
  program.add_argument("-o", "--output")
      .help("Output path for headless mode")
      .store_into(out_path);

  std::string backend_opt = "cpu";
  program.add_argument("--backend")
      .help("Select backend: cpu or cuda (default: cpu)")
      .store_into(backend_opt);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    spdlog::error("Fail to parse cli args. Reason: {}", err.what());
    return 1;
  }

  std::optional<SimConfig> sim_config = std::nullopt;
  if (!config_path.empty()) {
    sim_config = parse_config(config_path);
    // TODO: sanitize simulation config here
    if (!sim_config) {
      spdlog::error("Fail to parse config file {}.", config_path);
    } else {
      spdlog::info("Load config file {}.", config_path);
    }
  }

  // Normalize backend option
  for (auto& ch : backend_opt) ch = static_cast<char>(::tolower(ch));
  silk::Backend backend_sel = silk::Backend::Cpu;
  if (backend_opt == "cpu") {
    backend_sel = silk::Backend::Cpu;
  } else if (backend_opt == "cuda") {
    backend_sel = silk::Backend::Gpu;
  } else {
    spdlog::error("Invalid backend '{}'. Allowed: cpu, cuda.", backend_opt);
    return 1;
  }

  if (is_headless) {
    if (!sim_config) {
      spdlog::error(
          "Please provide a config json through -c flag in headless mode.");
      return 1;
    }
    spdlog::info("Headless mode.");
    headless_run(*sim_config, out_path, backend_sel);
    return 0;
  } else {
    spdlog::info("GUI mode.");

    py::options::buildGui = false;
    py::init();
    py::view::setUpDir(polyscope::UpDir::ZUp);
    py::view::setFrontDir(polyscope::FrontDir::XFront);
    py::options::groundPlaneMode = py::GroundPlaneMode::None;

    Demo demo_app;
    if (!demo_app.set_backend(backend_sel)) {
      return 1;
    }
    if (sim_config) {
      demo_app.apply_config(*sim_config);
    }
    demo_app.run();
  }

  return 0;
}
