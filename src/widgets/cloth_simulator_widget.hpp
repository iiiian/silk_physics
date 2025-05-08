#pragma once

#include <Eigen/Core>
#include <chrono>

#include "../gui_helper.hpp"
#include "../solver.hpp"

class ClothSimulatorWidget : public IWidget {
  UIContext& ui_ctx_;
  EngineContext& engine_ctx_;
  RMatrixX3f original_V;
  std::chrono::steady_clock::time_point prev_update_time;
  float gravity = 1;
  ClothSolver solver;
  
  // For drag selection
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_dragging_ = false;

  void enter_sim_mode();
  void leave_sim_mode();
  void compute_cloth(float elapse_sec);
  void handle_drag_selection();

 public:
  int target_fps_ = 60;

  explicit ClothSimulatorWidget(UIContext& ui_context,
                                EngineContext& engine_context);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};
