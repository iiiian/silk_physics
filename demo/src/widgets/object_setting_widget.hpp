#pragma once

#include <polyscope/point_cloud.h>

#include <Eigen/Core>
#include <glm/glm.hpp>

#include "../gui_utils.hpp"

class ObjectSettingWidget : public IWidget {
 private:
  Context& ctx_;

  // Selector sphere visualization
  polyscope::PointCloud* selector_sphere_ = nullptr;
  glm::vec3 selector_center_ = {0, 0, 0};
  float selector_radius_ = 0.1f;

  // Input tracking
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_mouse_on_surface_ = false;

  int pick_radius_ = 1;

 public:
  explicit ObjectSettingWidget(Context& context);
  void draw() override;

 private:
  void enter_paint_mode();
  void leave_paint_mode();
  void handle_paint_input();
};
