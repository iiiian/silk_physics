#pragma once

#include <Eigen/Core>
#include <chrono>
#include <silk/silk.hpp>

#include "../gui_helper.hpp"

class SimulatorWidget : public IWidget {
  Context& ctx_;

  std::chrono::steady_clock::time_point prev_update_time_;
  float sim_fps_ = 0.0f;

  // For drag selection
  Object* selected_obj = nullptr;
  bool is_first_click = false;
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};

  void init_cloth(Object& obj);
  void init_obstacle(Object& obj);
  void enter_sim_mode();
  void leave_sim_mode();
  void update_pin(const Object& obj);
  void update_pos(Object& obj);
  void solver_step(int substep);
  void handle_drag_selection();

 public:
  explicit SimulatorWidget(Context& context);
  void draw() override;
};
