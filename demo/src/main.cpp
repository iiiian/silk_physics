#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>
#include <argparse/argparse.hpp>
#include <string>
#include "demo.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;
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
  program.add_argument("-H", "--headless")
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
    try {
      std::ifstream in(config_path);
      if (!in) {
        spdlog::error("Cannot open config file: {}", config_path);
        return 1;
      }
      json cfg = json::parse(in, nullptr, true, true);

      if (!cfg.contains("paths") || !cfg["paths"].is_object()) { spdlog::error("Config error: missing object 'paths'"); return 1; }
      if (!cfg.contains("global") || !cfg["global"].is_object()) { spdlog::error("Config error: missing object 'global'"); return 1; }
      if (!cfg.contains("cloth") || !cfg["cloth"].is_object()) { spdlog::error("Config error: missing object 'cloth'"); return 1; }
      if (!cfg.contains("collision") || !cfg["collision"].is_object()) { spdlog::error("Config error: missing object 'collision'"); return 1; }

      int total_steps = -1;
      double dt = -1.0;
      if (cfg["global"].contains("total_steps") && cfg["global"]["total_steps"].is_number_integer())
        total_steps = cfg["global"]["total_steps"].get<int>();
      if (cfg["global"].contains("time_step") && cfg["global"]["time_step"].is_number())
        dt = cfg["global"]["time_step"].get<double>();
      if (total_steps <= 0) { spdlog::error("Config error: global.total_steps must be a positive integer"); return 1; }
      if (dt <= 0.0) { spdlog::error("Config error: global.time_step must be a positive number"); return 1; }

      if (!is_headless) {
        if (cfg["global"].contains("headless") && cfg["global"]["headless"].is_boolean())
          is_headless = cfg["global"]["headless"].get<bool>();
        else if (cfg.contains("headless") && cfg["headless"].is_boolean())
          is_headless = cfg["headless"].get<bool>();
      }

      if (cloth_model_path.empty()) {
        if (cfg["paths"].contains("mesh") && cfg["paths"]["mesh"].is_string())
          cloth_model_path = cfg["paths"]["mesh"].get<std::string>();
        else if (cfg.contains("cloth_model_path") && cfg["cloth_model_path"].is_string())
          cloth_model_path = cfg["cloth_model_path"].get<std::string>();
      }

      auto need_num = [](const json& obj, const char* key, const char* ctx, double min, bool inclusive) -> bool {
        if (!obj.contains(key) || !obj[key].is_number()) { spdlog::error("Config error: {}.'{}' missing or not a number", ctx, key); return false; }
        double v = obj[key].get<double>();
        if (inclusive ? (v < min) : (v <= min)) { spdlog::error("Config error: {}.'{}' must be {} {}", ctx, key, inclusive ? ">=" : ">", min); return false; }
        return true;
      };
      if (!need_num(cfg["cloth"], "mass", "cloth", 0.0, false)) return 1;
      if (!need_num(cfg["cloth"], "stiffness_stretch", "cloth", 0.0, false)) return 1;
      if (!need_num(cfg["cloth"], "stiffness_bending", "cloth", 0.0, true)) return 1;
      if (!need_num(cfg["cloth"], "damping", "cloth", 0.0, true)) return 1;

      if (!cfg["collision"].contains("enabled") || !cfg["collision"]["enabled"].is_boolean()) {
        spdlog::error("Config error: collision.enabled must be boolean"); return 1;
      }
      if (!need_num(cfg["collision"], "thickness", "collision", 0.0, true)) return 1;
      if (!cfg["collision"].contains("self_collision") || !cfg["collision"]["self_collision"].is_boolean()) {
        spdlog::error("Config error: collision.self_collision must be boolean"); return 1;
      }
      // Optional: pin vertices
      if (cfg.contains("pins") && cfg["pins"].is_array()) {
        spdlog::info("Found {} pin(s) in config", cfg["pins"].size());
        // TODO: handle pin vertices in solver
      }

      // Optional: transform
      if (cfg.contains("transform") && cfg["transform"].is_object()) {
        auto t = cfg["transform"];
        if (t.contains("scale") && t["scale"].is_number()) {
          double scale = t["scale"].get<double>();
          spdlog::info("Transform: scale = {}", scale);
        }
        if (t.contains("rotation_euler_deg") && t["rotation_euler_deg"].is_array()) {
          spdlog::info("Transform: rotation = {}", t["rotation_euler_deg"].dump());
        }
        if (t.contains("translation") && t["translation"].is_array()) {
          spdlog::info("Transform: translation = {}", t["translation"].dump());
        }
        // TODO: apply transform to model
      }


      spdlog::info("Config loaded from {}", config_path);
      spdlog::info("  mesh = {}", cloth_model_path.empty() ? "<none>" : cloth_model_path);
      spdlog::info("  steps = {}, dt = {}, headless = {}", total_steps, dt, is_headless);
    } catch (const std::exception& e) {
      spdlog::error("JSON error: {}", e.what());
      return 1;
    }
  }

  if (is_headless) {
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
    spdlog::info("Attempting to load model from command line argument: {}", cloth_model_path);
    if (!demo_app.load_model_from_path(cloth_model_path)) {
      spdlog::warn("Could not load initial model specified via command line: {}. Application will continue without it.", cloth_model_path);
    } else {
      spdlog::info("Successfully loaded initial model: {}", cloth_model_path);
    }
  } else {
    spdlog::info("No initial model path provided via command line arguments. Load a model using the GUI.");
  }

  demo_app.run();
  return 0;
}
