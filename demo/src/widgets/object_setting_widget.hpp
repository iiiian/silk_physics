#pragma once

#include <polyscope/point_cloud.h>

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <memory>

#include "../gui_helper.hpp"
#include "../kd_tree.hpp"

class ObjectSettingWidget : public IWidget {
 private:
  Context& ctx_;

 public:
  explicit ObjectSettingWidget(Context& context);
  void draw() override;

 private:
  // Selector sphere visualization
  polyscope::PointCloud* selector_sphere_ = nullptr;
  glm::vec3 selector_center_ = {0, 0, 0};
  float selector_radius_ = 0.1f;

  // Input tracking
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_mouse_on_surface_ = false;

  // Spatial data structures
  std::unique_ptr<KDTree> kd_tree_;

  void enter_paint_mode();
  void leave_paint_mode();
  void update_selection_visual();
  void select_vertices_in_sphere(bool add_to_selection);
  void handle_paint_input();

  void draw_type_combo();
  void draw_cloth_setting();
  void draw_collision_setting();
  void draw_pin_group_setting();
};
