#include <igl/bounding_box_diagonal.h>
#include <igl/readOFF.h>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
// #include <portable-file-dialogs.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>
#include <functional>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "common_types.hpp"
#include "kd_tree.hpp"

class Demo {
  // Core mesh data
  eg::MatrixX3d verts_;
  eg::MatrixX3i faces_;
  float mesh_diag_ = 0;
  py::SurfaceMesh* surface_ = nullptr;

  KDTree kd_tree;

  // Smooth mesh state
  eg::MatrixX3d smooth_verts_;
  bool has_smoothed_mesh_ = false;
  bool is_showing_smooth_mesh_ = false;

  // Vertex selection state
  bool is_paint_mode_ = false;
  // Visual aid for painting
  py::PointCloud* selector_sphere_ = nullptr;
  eg::Vector3d selector_center_ = eg::Vector3d::Zero();
  float selector_radius_ = 0.1f;
  // Initialize to invalid pos to force first update
  ImVec2 prev_mouse_pos_ = {-1.0f, -1.0f};
  bool is_mouse_on_surface_ = false;

  // Segment management state
  std::string new_segment_name_ = "Segment 0";
  std::unordered_set<int> current_selection_;
  std::vector<Segment> segments_;
  // Next available ID for new segments
  long segment_next_id_ = 0;
  int current_segment_idx_ = -1;

  // Mesh deformation state
  bool is_deformation_mode_ = false;
  // Result of deformation + detail reconstruction
  eg::MatrixX3d deform_verts_;
  MeshDetail mesh_detail_;
  MeshDeformationSolver deformation_solver_;

  // --- Private Helper Methods ---

  // Resets the application state to its initial condition, typically after
  // loading a new mesh.
  void reset_state() {
    if (is_paint_mode_) {
      leave_paint_mode();
    }
    if (is_deformation_mode_) {
      leave_deformation_mode();
    }

    // Clear mesh data (will be replaced by loaded data)
    verts_.resize(0, 3);
    faces_.resize(0, 3);
    mesh_diag_ = 0;
    py::removeStructure("mesh");
    surface_ = nullptr;

    // Reset smoothing state
    smooth_verts_.resize(0, 3);
    has_smoothed_mesh_ = false;
    is_showing_smooth_mesh_ = false;

    // Reset selection state
    current_selection_.clear();
    selector_center_ = eg::Vector3d::Zero();
    selector_radius_ = 0.1f;
    is_mouse_on_surface_ = false;

    // Reset segment state
    segments_.clear();
    segment_next_id_ = 0;
    current_segment_idx_ = -1;
    new_segment_name_ = "Segment 0";

    // Reset deformation state
    deform_verts_.resize(0, 3);

    // Reset KD-Tree
    kd_tree.clear();

    spdlog::info("Application state reset.");
  }

  // Switches the displayed mesh in Polyscope to the smoothed version.
  void show_smooth_mesh() {
    assert(has_smoothed_mesh_ &&
           "Smooth mesh must be computed before showing.");
    assert(smooth_verts_.rows() == verts_.rows() &&
           "Smooth mesh vertex count mismatch.");
    assert(surface_ && "Surface mesh must exist.");

    is_showing_smooth_mesh_ = true;
    surface_->updateVertexPositions(smooth_verts_);
    spdlog::debug("Showing smoothed mesh.");
  }

  // Switches the displayed mesh in Polyscope to the original detailed version.
  void show_detail_mesh() {
    assert(verts_.rows() > 0 && "Original vertices must exist.");
    assert(surface_ && "Surface mesh must exist.");

    is_showing_smooth_mesh_ = false;
    surface_->updateVertexPositions(verts_);
    spdlog::debug("Showing detail mesh.");
  }

  // Creates a new segment from the current vertex selection.
  void create_segment() {
    assert(!current_selection_.empty() && "Cannot create an empty segment.");
    assert(segment_next_id_ >= 0 && "Segment next ID is invalid.");

    Segment segment;
    segment.name = new_segment_name_;
    segment.id = segment_next_id_++;
    segment.verts = current_selection_;

    segments_.push_back(segment);
    // Select the new segment
    current_segment_idx_ = static_cast<int>(segments_.size() - 1);
    spdlog::info(fmt::format("Created segment '{}' with {} vertices.",
                             segment.name, segment.verts.size()));
  }

  // Activates paint mode for vertex selection.

  // Selects vertices within the brush sphere.

  // --- File I/O Methods ---

  void load_off() {
    auto file_selection =
        pfd::open_file("Select OFF mesh", "./", {"OFF files", "*.off"})
            .result();
    if (file_selection.empty()) {
      spdlog::info("Mesh loading cancelled by user.");
      return;
    }
    load_off_from_path(file_selection[0]);
  }

  void save_segments() {
    if (segments_.empty()) {
      spdlog::warn("There are no segments to save.");
      return;
    }

    std::string path =
        pfd::save_file("Save Segments", "./segment_data.seg").result();
    if (path.empty()) {
      spdlog::info("Segment saving cancelled by user.");
      return;
    }

    spdlog::info("Attempting to save {} segments to {}", segments_.size(),
                 path);
    try {
      serialize_segments(path, segments_);
      spdlog::info(fmt::format("Segments saved successfully to {}.", path));
    } catch (const std::exception& e) {
      spdlog::error(fmt::format("Failed to save segments to file {}: {}", path,
                                e.what()));
    }
  }

  void load_segments() {
    if (!surface_) {
      spdlog::warn("Please load a mesh first before loading segments.");
      return;
    }

    auto file_selection = pfd::open_file("Load Segments", "./").result();
    if (file_selection.empty()) {
      spdlog::info("Segment loading cancelled by user.");
      return;
    }
    const std::string& path = file_selection[0];

    spdlog::info("Attempting to load segments from {}", path);
    try {
      std::vector<Segment> loaded_segments = deserialize_segments(path);
      if (loaded_segments.empty()) {
        spdlog::warn(
            fmt::format("Loaded segment file was empty or invalid: {}", path));
        return;
      }

      segments_ = std::move(loaded_segments);

      // Update state based on loaded segments
      // Select first segment if exists
      current_segment_idx_ = segments_.empty() ? -1 : 0;
      if (current_segment_idx_ != -1) {
        current_selection_ = segments_[0].verts;
        update_selection_visual();
      } else {
        // Clear visual selection if no segments loaded
        current_selection_.clear();
        update_selection_visual();
      }

      segment_next_id_ = segments_.size();
      new_segment_name_ = fmt::format("Segment {}", segment_next_id_);

      spdlog::info(
          fmt::format("Segments loaded successfully from {}. Found {} "
                      "segments. Next ID: {}",
                      path, segments_.size(), segment_next_id_));

      // Invalidate smoothing as segments changed
      has_smoothed_mesh_ = false;
      is_showing_smooth_mesh_ = false;

    } catch (const std::exception& e) {
      spdlog::error(fmt::format("Failed to load segments from file {}: {}",
                                path, e.what()));
    }
  }

  // --- GUI Drawing Methods ---

  void draw_mesh_loader_widget() {
    // Disable loading if busy with painting or deformation
    ImGui::BeginDisabled(is_paint_mode_ || is_deformation_mode_);
    if (ImGui::CollapsingHeader("Load Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Load .off Mesh")) {
        load_off();
      }
    }
    ImGui::EndDisabled();
  }

  void draw_selector_widget() {}

  void draw_segment_manager_widget() {
    if (ImGui::CollapsingHeader("Segments", ImGuiTreeNodeFlags_DefaultOpen)) {
      // --- Segment File I/O ---
      ImGui::BeginDisabled(!surface_ || is_paint_mode_ || is_deformation_mode_);

      // Save Segments Button
      // Disable save if no segments exist
      ImGui::BeginDisabled(segments_.empty());
      if (ImGui::Button("Save Segments")) {
        save_segments();
      }
      ImGui::EndDisabled();

      ImGui::SameLine();

      if (ImGui::Button("Load Segments")) {
        load_segments();
      }

      ImGui::EndDisabled();  // End of I/O disable block

      ImGui::Separator();

      // --- Segment List ---
      ImGui::Text("Loaded Segments:");
      ImGui::BeginChild("SegmentList", ImVec2(0, 150), true);
      for (size_t i = 0; i < segments_.size(); ++i) {
        const Segment& segment = segments_[i];

        // Use unique segment ID for ImGui
        ImGui::PushID(static_cast<int>(segment.id));
        bool is_selected = (current_segment_idx_ == static_cast<int>(i));

        // Make the segment selectable
        if (ImGui::Selectable(segment.name.c_str(), is_selected)) {
          if (current_segment_idx_ != static_cast<int>(i)) {
            current_segment_idx_ = static_cast<int>(i);
            // Update selection to match segment
            current_selection_ = segment.verts;
            update_selection_visual();
            spdlog::debug("Selected segment '{}' (Index: {})", segment.name, i);
          }
        }

        // Display vertex count next to the selectable item
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu vertices)", segment.verts.size());

        ImGui::PopID();
      }
      ImGui::EndChild();  // End SegmentList
    }
  }

  // Handles mouse interaction for painting selection when in paint mode.

  // Main ImGui drawing callback, called every frame by Polyscope.
  void draw_imgui() {
    draw_mesh_loader_widget();
    draw_selector_widget();
    draw_segment_manager_widget();

    handle_paint_input();
  }

 public:
  Demo() {
    py::init();
    py::state::userCallback = [this]() { this->draw_imgui(); };
    // spdlog::set_level(spdlog::level::debug);
    spdlog::info("Application initialized.");
  }

  // Starts the Polyscope event loop.
  void run() {
    spdlog::info("Starting Polyscope main loop...");
    py::show();
    spdlog::info("Polyscope main loop finished.");
  }

  // Loads an OFF mesh from the given file path and initializes the application
  // state.
  void load_off_from_path(const std::string& path) {
    spdlog::info("Attempting to load mesh from: {}", path);

    // --- Reset State Before Loading ---
    reset_state();

    // --- Read Mesh Data ---
    eg::MatrixX3d V;
    eg::MatrixX3i F;
    if (!igl::readOFF(path, V, F)) {
      spdlog::error(fmt::format("Failed to read OFF file: {}", path));
      return;
    }

    // --- Validate Mesh Data ---
    if (V.rows() == 0) {
      spdlog::error(fmt::format("Mesh file has no vertices: {}", path));
      return;
    }
    if (F.rows() == 0) {
      spdlog::warn(fmt::format("Loaded mesh has no faces: {}", path));
    }

    // --- Update Application State ---
    verts_ = std::move(V);
    faces_ = std::move(F);
    mesh_diag_ = igl::bounding_box_diagonal(verts_);
    surface_ = py::registerSurfaceMesh("mesh", verts_, faces_);
    if (!surface_) {
      spdlog::error("Failed to register mesh with Polyscope.");
      reset_state();
      return;
    }

    kd_tree.init(&verts_);

    // Initialize selector properties based on the new mesh
    selector_center_ = verts_.colwise().mean();
    selector_radius_ = std::max(0.001f, mesh_diag_ * 0.05f);
    update_selection_visual();

    spdlog::info(
        fmt::format("Successfully loaded mesh '{}' ({} vertices, {} faces).",
                    path, verts_.rows(), faces_.rows()));
    py::view::resetCameraToHomeView();
  }
};

int main(int argc, char** argv) {
  try {
    Demo app;
    if (argc == 2) {
      app.load_off_from_path(argv[1]);
    }
    app.run();
  } catch (const std::exception& e) {
    spdlog::critical("Unhandled exception: {}", e.what());
    return 1;
  }

  return 0;
}
