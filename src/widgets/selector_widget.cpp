#include "selector_widget.hpp"

#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <glm/glm.hpp>

#include "../gui_helper.hpp"

namespace py = polyscope;
namespace eg = Eigen;

void SelectorWidget::enter_paint_mode() {
  assert(ctx_.ui_mode == UIMode::Normal);

  ctx_.ui_mode = UIMode::Paint;
  py::state::doDefaultMouseInteraction = false;

  ctx_.p_surface->setSelectionMode(py::MeshSelectionMode::Auto);

  selector_sphere_ = py::registerPointCloud(
      "selector_sphere", std::vector<glm::vec3>{selector_center_});
  selector_sphere_->setPointRadius(selector_radius_, false);
  selector_sphere_->setPointColor({1.0f, 0.5f, 0.0f});
  selector_sphere_->setTransparency(0.5f);

  spdlog::info("Entered paint mode.");
}

void SelectorWidget::leave_paint_mode() {
  assert(ctx_.ui_mode == UIMode::Paint);

  ctx_.ui_mode = UIMode::Normal;
  py::state::doDefaultMouseInteraction = true;
  py::removeStructure("selector_sphere");
  selector_sphere_ = nullptr;

  spdlog::info("Left paint mode.");
}

void SelectorWidget::update_selection_visual() {
  assert(ctx_.p_surface);
  assert(ctx_.p_surface->nVertices() != 0);

  auto s = ctx_.p_surface;
  std::vector<double> indicator(s->nVertices(), 0.0);
  for (int idx : ctx_.selection) {
    indicator[idx] = 1.0;
  }
  s->addVertexScalarQuantity("selection", indicator)->setEnabled(true);
}

void SelectorWidget::select_vertices_in_sphere(bool add_to_selection) {
  assert(ctx_.ui_mode == UIMode::Paint);
  assert(ctx_.p_surface->nVertices() != 0);

  eg::Vector3f center(selector_center_.x, selector_center_.y,
                      selector_center_.z);
  std::vector<eg::Index> matches =
      kd_tree_.find_neighbors(center, selector_radius_);

  size_t changed_count = 0;
  if (add_to_selection) {
    for (auto match : matches) {
      if (ctx_.selection.insert(match).second) {
        changed_count++;
      }
    }
  } else {
    for (auto match : matches) {
      if (ctx_.selection.erase(match) > 0) {
        changed_count++;
      }
    }
  }

  if (changed_count > 0) {
    update_selection_visual();
    spdlog::trace("Selection updated, {} vertices changed.", changed_count);
  }
}

void SelectorWidget::handle_paint_input() {
  if (!(ctx_.ui_mode == UIMode::Paint) || !ctx_.p_surface) {
    return;
  }

  ImVec2 mouse_pos = ImGui::GetMousePos();
  float delta_x = std::abs(mouse_pos.x - prev_mouse_pos_.x);
  float delta_y = std::abs(mouse_pos.y - prev_mouse_pos_.y);
  bool mouse_moved = (delta_x > 1 || delta_y > 1);

  if (mouse_moved) {
    prev_mouse_pos_ = mouse_pos;
    py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});

    is_mouse_on_surface_ = pick.isHit;
    if (pick.isHit && pick.structure == ctx_.p_surface) {
      selector_center_ = pick.position;
      assert(selector_sphere_);
      selector_sphere_->updatePointPositions(
          std::vector<glm::vec3>{selector_center_});
    }
  }

  if (!is_mouse_on_surface_) {
    return;
  }

  bool left_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool right_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);
  if (left_mouse_down) {
    select_vertices_in_sphere(true);
  } else if (right_mouse_down) {
    select_vertices_in_sphere(false);
  }
}

void SelectorWidget::rebuild_kd_tree() {
  assert(ctx_.p_surface);

  auto& raw_verts = ctx_.p_surface->vertexPositions;
  verts_.resize(raw_verts.size(), 3);
  for (size_t i = 0; i < raw_verts.size(); ++i) {
    auto v = raw_verts.getValue(i);
    verts_(i, 0) = v.x;
    verts_(i, 1) = v.y;
    verts_(i, 2) = v.z;
  }
  kd_tree_.init(&verts_);
}

SelectorWidget::SelectorWidget(UIContext& context) : ctx_(context) {}

EventFlag SelectorWidget::draw() {
  ImGui::BeginDisabled(!ctx_.p_surface || !(ctx_.ui_mode == UIMode::Normal ||
                                            ctx_.ui_mode == UIMode::Paint));

  if (need_rebuild_kdtree_ && ctx_.ui_mode == UIMode::Paint) {
    rebuild_kd_tree();
    need_rebuild_kdtree_ = false;
  }

  if (ImGui::CollapsingHeader("Vertex Selector",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    // Brush Radius Slider
    ImGui::SliderFloat("Brush Radius", &selector_radius_,
                       0.001f * ctx_.mesh_diag, ctx_.mesh_diag * 0.2f, "%.4f");

    if (selector_sphere_) {
      selector_sphere_->setPointRadius(selector_radius_, false);
    }

    if (ImGui::Button(ctx_.ui_mode == UIMode::Paint ? "Stop Painting"
                                                    : "Start Painting")) {
      if (ctx_.ui_mode == UIMode::Paint) {
        leave_paint_mode();
      } else {
        enter_paint_mode();
      }
    }

    ImGui::SameLine();

    ImGui::BeginDisabled(ctx_.selection.empty());
    if (ImGui::Button("Clear Selection")) {
      ctx_.selection.clear();
      update_selection_visual();
      spdlog::debug("Selection cleared.");
    }
    ImGui::EndDisabled();

    ImGui::Text("Selected vertices: %zu", ctx_.selection.size());
  }
  ImGui::EndDisabled();

  handle_paint_input();
  return EventFlag::NoEvent;
}

void SelectorWidget::on_event(EventFlag events) {
  if (raw(events & EventFlag::MeshChange)) {
    need_rebuild_kdtree_ = true;
  }
}
