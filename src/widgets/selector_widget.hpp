#pragma once

#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <nanoflann.hpp>

#include "../gui_helper.hpp"

class SelectorWidget : public IWidget {
  AppContext& ctx_;

  // Selector sphere visualization
  polyscope::PointCloud* selector_sphere_ = nullptr;
  glm::vec3 selector_center_ = {0, 0, 0};
  float selector_radius_ = 0.1f;

  // Input tracking
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_mouse_on_surface_ = false;

  // Spatial data structures
  Eigen::MatrixX3f verts_;
  nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixX3f, 3> kd_tree_{3, verts_};

  void enter_paint_mode();
  void leave_paint_mode();
  void update_selection_visual();
  void select_vertices_in_sphere(bool add_to_selection);
  void handle_paint_input();

 public:
  explicit SelectorWidget(AppContext& context);
  void draw() override;
  void on_event(Flags<Event> events) override;
};
