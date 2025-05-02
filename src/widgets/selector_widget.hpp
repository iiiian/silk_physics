#ifndef SELECTOR_WIDGET_HPP
#define SELECTOR_WIDGET_HPP

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
public:
    explicit SelectorWidget(AppContext& context);
    
    //==== Public Interface ====//
    void draw() override;
    void on_event(Flags<Event> events) override;

private:
    //==== Member Variables ====//
    AppContext& ctx_;

    // Selector sphere visualization
    py::PointCloud* selector_sphere_ = nullptr;
    glm::vec3 selector_center_ = {0, 0, 0};
    float selector_radius_ = 0.1f;

    // Input tracking
    ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
    bool is_mouse_on_surface_ = false;

    // Spatial data structures
    eg::MatrixX3f verts_;
    na::KDTreeEigenMatrixAdaptor<eg::MatrixX3f, 3> kd_tree_{3, verts_};

    //==== Paint Mode Management ====//
    void enter_paint_mode();
    void leave_paint_mode();

    //==== Selection Operations ====//
    void update_selection_visual();
    void select_vertices_in_sphere(bool add_to_selection);
    void handle_paint_input();
};

#endif // SELECTOR_WIDGET_HPP
