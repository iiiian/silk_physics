#pragma once

#include <Eigen/Core>
#include <chrono>
#include <silk/silk.hpp>

#include "../gui_helper.hpp"

class SimulatorWidget : public IWidget {
 private:
  Context& ctx_;

  std::chrono::steady_clock::time_point prev_update_time_;
  float sim_fps_ = 0.0f;

 public:
  explicit SimulatorWidget(Context& context);
  void draw() override;

 private:
  void enter_sim_mode();
  void leave_sim_mode();
  void solver_step(int substep);
};
