#include "simulator_widget.hpp"

#include <polyscope/polyscope.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>
#include <cmath>

namespace py = polyscope;

void SimulatorWidget::enter_sim_mode() {
  silk::Result r = ctx_.silk_world.solver_reset();
  if (!r) {
    spdlog::error("Solver reset fail. Error: {}", r.to_string());
  }

  // init all objects via IObject
  for (auto& obj : ctx_.objects) {
    if (!obj->init_sim()) {
      spdlog::error("init_sim failed for object '{}'", obj->get_name());
      return;
    }
  }

  r = ctx_.silk_world.set_global_config(ctx_.global_config);
  if (!r) {
    spdlog::error("Set global config failed. Error: {}", r.to_string());
    return;
  }

  py::state::doDefaultMouseInteraction = false;
  prev_update_time_ = std::chrono::steady_clock::now();
  ctx_.ui_mode = UIMode::Sim;
  spdlog::info("Enter sim mode");
}

void SimulatorWidget::leave_sim_mode() {
  for (auto& obj : ctx_.objects) {
    if (!obj->exit_sim()) {
      spdlog::error("Object {} fail to exit simulation mode", obj->get_name());
    }
  }

  py::state::doDefaultMouseInteraction = true;
  ctx_.ui_mode = UIMode::Normal;
  spdlog::info("Leave sim mode");
}

void SimulatorWidget::solver_step(int substep) {
  // pre-step hooks
  for (auto& pobj : ctx_.objects) {
    pobj->sim_step_pre();
  }

  for (int i = 0; i < substep; ++i) {
    silk::Result r = ctx_.silk_world.solver_step();
    if (!r) {
      spdlog::error("Solver step fail. Error: {}", r.to_string());
      leave_sim_mode();
      return;
    }
  }

  // post-step hooks
  for (auto& pobj : ctx_.objects) {
    pobj->sim_step_post();
  }
}

SimulatorWidget::SimulatorWidget(Context& context) : ctx_(context) {}

void SimulatorWidget::draw() {
  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Simulation FPS: %f", sim_fps_);

    ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal &&
                         ctx_.ui_mode != UIMode::Sim);
    if (ImGui::Button(ctx_.ui_mode == UIMode::Sim ? "Stop Simulation"
                                                  : "Start Simulation")) {
      if (ctx_.ui_mode == UIMode::Sim) {
        leave_sim_mode();
      } else {
        enter_sim_mode();
      }
    }
    ImGui::EndDisabled();
  }

  if (ctx_.ui_mode == UIMode::Sim) {
    using std::chrono::microseconds;
    using std::chrono::steady_clock;

    auto now = steady_clock::now();
    double elapsed_us =
        std::chrono::duration_cast<microseconds>(now - prev_update_time_)
            .count();

    double dt_us = 1e6f * ctx_.global_config.dt;
    int substep = static_cast<int>(std::floor(elapsed_us / dt_us));
    if (substep <= 0) {
      return;
    }

    spdlog::info("Solver step {}", substep);

    auto s0 = steady_clock::now();
    solver_step(substep);
    auto s1 = steady_clock::now();

    double sim_us = std::chrono::duration_cast<microseconds>(s1 - s0).count();
    if (sim_us <= 0.0f) {
      sim_us = 1.0f;
    }
    sim_fps_ = static_cast<float>(substep * 1e6f / sim_us);

    prev_update_time_ = steady_clock::now();
  }
}
