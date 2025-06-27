#pragma once

#include <Eigen/Core>
#include <chrono>
#include <optional>
#include <silk/silk.hpp>

#include "../eigen_alias.hpp"
#include "../gui_helper.hpp"

class ClothSimulatorWidget : public IWidget {
  UIContext& ui_ctx_;
  EngineContext& engine_ctx_;

  Verts original_V;
  std::chrono::steady_clock::time_point prev_update_time_;

  silk::ClothConfig cloth_cfg_;
  std::optional<silk::Handle> cloth_handle_;

  silk::World solver_;
  float gravity_ = 10;
  int solver_thread_num_ = 4;
  int solver_max_iter_ = 10;
  int solver_low_freq_mode_num_ = 30;

  // For drag selection
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};

  void enter_sim_mode();
  void leave_sim_mode();
  void compute_cloth(float elapse_sec);
  void handle_drag_selection();

 public:
  int target_fps_ = 180;

  explicit ClothSimulatorWidget(UIContext& ui_context,
                                EngineContext& engine_context);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};
