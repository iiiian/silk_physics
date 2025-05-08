#pragma once

#include <Eigen/Core>
#include <chrono>

#include "../gui_helper.hpp"
#include "../solver.hpp"

class ClothSimulatorWidget : public IWidget {
  UIContext& ui_ctx_;
  EngineContext& engine_ctx_;
  std::chrono::steady_clock::time_point prev_update_time;
  ClothSolver solver;

  void enter_sim_mode();
  void leave_sim_mode();
  void compute_cloth();

 public:
  int target_fps_ = 60;

  explicit ClothSimulatorWidget(UIContext& ui_context,
                                EngineContext& engine_context);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};
