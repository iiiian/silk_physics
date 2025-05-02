#include <polyscope/point_cloud.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <nanoflann.hpp>

#include "../gui_helper.hpp"

namespace py = polyscope;
namespace na = nanoflann;
namespace eg = Eigen;

class SelectorWidget : public IWidget {
  AppContext& ctx_;

  // selector sphere
  py::PointCloud* selector_sphere_ = nullptr;
  glm::vec3 selector_center_ = {0, 0, 0};
  float selector_radius_ = 0.1f;

  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_mouse_on_surface_ = false;

  // save a copy of mesh for kd tree
  eg::MatrixX3f verts_;
  na::KDTreeEigenMatrixAdaptor<eg::MatrixX3f, 3> kd_tree_{3, verts_};

  void enter_paint_mode() {
    assert(ctx_.ui_mode == UIMode::Normal);

    ctx_.ui_mode = UIMode::Paint;
    // Disable Polyscope's camera controls
    py::state::doDefaultMouseInteraction = false;

    // Create the visual sphere for the brush
    selector_sphere_ =
        py::registerPointCloud("selector_sphere", selector_center_);
    selector_sphere_->setPointRadius(selector_radius_, false);
    selector_sphere_->setPointColor({1.0f, 0.5f, 0.0f});  // Orange
    selector_sphere_->setTransparency(0.5f);

    spdlog::info("Entered paint mode.");
  }

  // Deactivates paint mode.
  void leave_paint_mode() {
    assert(ctx_.ui_mode == UIMode::Paint);

    ctx_.ui_mode = UIMode::Normal;
    py::state::doDefaultMouseInteraction = true;
    py::removeStructure("selector_sphere");
    selector_sphere_ = nullptr;

    spdlog::info("Left paint mode.");
  }

  // Updates the vertex scalar quantity in Polyscope to visualize the current
  // selection.
  void update_selection_visual() {
    assert(ctx_.p_surface);
    assert(ctx_.p_surface->nVertices() != 0);

    auto s = ctx_.p_surface;
    // Create a scalar quantity: 1.0 for selected, 0.0 otherwise
    std::vector<double> indicator(s->nVertices(), 0.0);
    for (int idx : ctx_.selection) {
      indicator[idx] = 1.0;
    }
    s->addVertexScalarQuantity("selection", indicator)->setEnabled(true);
  }

  void handle_paint_input() {
    // Only process if in paint mode and a surface exists
    if (!(ctx_.ui_mode == UIMode::Paint) || !ctx_.p_surface) {
      return;
    }

    ImVec2 mouse_pos = ImGui::GetMousePos();

    // Check if mouse moved significantly to avoid constant
    // picking
    float delta_x = std::abs(mouse_pos.x - prev_mouse_pos_.x);
    float delta_y = std::abs(mouse_pos.y - prev_mouse_pos_.y);
    bool mouse_moved = (delta_x > 1 || delta_y > 1);

    bool left_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool right_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);

    // Perform picking and update sphere position if mouse moved
    if (mouse_moved) {
      prev_mouse_pos_ = mouse_pos;
      py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});

      is_mouse_on_surface_ = pick.isHit;
      // If the pick hit the registered surface mesh
      if (pick.isHit && pick.structure == ctx_.p_surface) {
        // Update the center of the selector sphere visual
        selector_center_ = pick.position;
        assert(selector_sphere_);
        selector_sphere_->updatePointPositions(selector_center_);
      }
    }

    if (!is_mouse_on_surface_) {
      return;
    }
    // If mouse buttons are down, perform selection/deselection
    if (left_mouse_down) {
      // Add to selection
      select_vertices_in_sphere(true);
    } else if (right_mouse_down) {
      // Remove from selection
      select_vertices_in_sphere(false);
    }
  }

  void select_vertices_in_sphere(bool add_to_selection) {
    assert(ctx_.ui_mode == UIMode::Paint);
    assert(ctx_.p_surface->nVertices() != 0);

    eg::Vector3f center(selector_center_.x, selector_center_.y,
                        selector_center_.z);
    std::vector<na::ResultItem<eg::Index, float>> matches;
    kd_tree_.index_->radiusSearch(center.data(), selector_radius_, matches);

    size_t changed_count = 0;
    if (add_to_selection) {
      for (auto match : matches) {
        if (ctx_.selection.insert(match.second).second) {
          changed_count++;
        }
      }
    } else {  // Erase from selection
      for (auto match : matches) {
        if (ctx_.selection.erase(match.second) > 0) {
          changed_count++;
        }
      }
    }

    // Only update visualization if something actually changed
    if (changed_count > 0) {
      update_selection_visual();
      spdlog::trace("Selection updated, {} vertices changed.", changed_count);
    }
  }

 public:
  SelectorWidget(AppContext& context) : ctx_(context) {}

  void draw() override {
    ImGui::BeginDisabled(!ctx_.p_surface || !(ctx_.ui_mode == UIMode::Normal ||
                                              ctx_.ui_mode == UIMode::Paint));
    if (ImGui::CollapsingHeader("Vertex Selector",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      // Brush Radius Slider
      // Adjust slider range based on mesh size
      ImGui::SliderFloat("Brush Radius", &selector_radius_,
                         0.001f * ctx_.mesh_diag, ctx_.mesh_diag * 0.2f,
                         "%.4f");
      // Update selector sphere radius immediately if it exists
      if (selector_sphere_) {
        selector_sphere_->setPointRadius(selector_radius_, false);
      }

      // Paint Mode Toggle Button
      if (ImGui::Button(ctx_.ui_mode == UIMode::Paint ? "Stop Painting"
                                                      : "Start Painting")) {
        if (ctx_.ui_mode == UIMode::Paint) {
          leave_paint_mode();
        } else {
          enter_paint_mode();
        }
      }

      ImGui::SameLine();

      // Clear Selection Button
      // Disable if nothing selected
      ImGui::BeginDisabled(ctx_.selection.empty());
      if (ImGui::Button("Clear Selection")) {
        ctx_.selection.clear();
        update_selection_visual();
        spdlog::debug("Selection cleared.");
      }
      ImGui::EndDisabled();

      ImGui::Text("Selected vertices: %zu", ctx_.selection.size());
    }
    ImGui::EndDisabled();  // End of selector widget block disable
  }

  void on_event(Flags<Event> events) override {
    if (!events.test(Event::MeshChange)) {
      return;
    }
    if (!ctx_.p_surface || ctx_.p_surface->nVertices() == 0) {
      return;
    }

    // rebuild kdtree on mesh change
    auto& raw_verts = ctx_.p_surface->vertexPositions;
    verts_.resize(raw_verts.size(), 3);
    for (size_t i = 0; i < raw_verts.size(); ++i) {
      auto v = raw_verts.getValue(i);
      verts_(i, 0) = v.x;
      verts_(i, 0) = v.y;
      verts_(i, 0) = v.z;
    }
    kd_tree_.index_->buildIndex();
  }
};
#include "selector_widget.hpp"

//==== Constructor ====//
SelectorWidget::SelectorWidget(AppContext& context) 
    : ctx_(context) {}

//==== Public Methods ====//

void SelectorWidget::draw() {
    ImGui::BeginDisabled(!ctx_.p_surface || 
                        !(ctx_.ui_mode == UIMode::Normal || 
                          ctx_.ui_mode == UIMode::Paint));
    
    if (ImGui::CollapsingHeader("Vertex Selector", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Brush Radius Slider
        ImGui::SliderFloat("Brush Radius", &selector_radius_,
                          0.001f * ctx_.mesh_diag, ctx_.mesh_diag * 0.2f,
                          "%.4f");
        
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
}

void SelectorWidget::on_event(Flags<Event> events) {
    if (!events.test(Event::MeshChange)) {
        return;
    }
    if (!ctx_.p_surface || ctx_.p_surface->nVertices() == 0) {
        return;
    }

    // Rebuild kd-tree on mesh change
    auto& raw_verts = ctx_.p_surface->vertexPositions;
    verts_.resize(raw_verts.size(), 3);
    for (size_t i = 0; i < raw_verts.size(); ++i) {
        auto v = raw_verts.getValue(i);
        verts_(i, 0) = v.x;
        verts_(i, 1) = v.y;
        verts_(i, 2) = v.z;
    }
    kd_tree_.index_->buildIndex();
}

//==== Paint Mode Management ====//

void SelectorWidget::enter_paint_mode() {
    assert(ctx_.ui_mode == UIMode::Normal);

    ctx_.ui_mode = UIMode::Paint;
    py::state::doDefaultMouseInteraction = false;

    selector_sphere_ = py::registerPointCloud("selector_sphere", selector_center_);
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

//==== Selection Operations ====//

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

    eg::Vector3f center(selector_center_.x, selector_center_.y, selector_center_.z);
    std::vector<na::ResultItem<eg::Index, float>> matches;
    kd_tree_.index_->radiusSearch(center.data(), selector_radius_, matches);

    size_t changed_count = 0;
    if (add_to_selection) {
        for (auto match : matches) {
            if (ctx_.selection.insert(match.second).second) {
                changed_count++;
            }
        }
    } else {
        for (auto match : matches) {
            if (ctx_.selection.erase(match.second) > 0) {
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

    bool left_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool right_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);

    if (mouse_moved) {
        prev_mouse_pos_ = mouse_pos;
        py::PickResult pick = py::pickAtScreenCoords({mouse_pos.x, mouse_pos.y});

        is_mouse_on_surface_ = pick.isHit;
        if (pick.isHit && pick.structure == ctx_.p_surface) {
            selector_center_ = pick.position;
            assert(selector_sphere_);
            selector_sphere_->updatePointPositions(selector_center_);
        }
    }

    if (!is_mouse_on_surface_) {
        return;
    }

    if (left_mouse_down) {
        select_vertices_in_sphere(true);
    } else if (right_mouse_down) {
        select_vertices_in_sphere(false);
    }
}
