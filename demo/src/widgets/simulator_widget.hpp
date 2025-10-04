#pragma once

#include <Eigen/Core>
#include <chrono>
#include <glm/glm.hpp>
#include <silk/silk.hpp>

#include "../gui_utils.hpp"

class SimulatorWidget : public IWidget {
 private:
  Context& ctx_;

  std::chrono::steady_clock::time_point prev_update_time_;
  float sim_time_ = 0.0f;
  float sim_fps_ = 0.0f;

  // Drag tracking for pin dragging
  IObject* drag_object_ = nullptr;
  glm::vec3 drag_position_;
  ImVec2 prev_mouse_pos_ = {0, 0};

 public:
  explicit SimulatorWidget(Context& context);
  void draw() override;

 private:
  void enter_sim_mode();
  void leave_sim_mode();
  void solver_step(int substep);
  void handle_pin_dragging();
};
