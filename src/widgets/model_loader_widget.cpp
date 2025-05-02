#include "model_loader_widget.hpp"

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include <spdlog/spdlog.h>

namespace py = polyscope;
namespace eg = Eigen;

ModelLoaderWidget::ModelLoaderWidget(UIContext& ui_context,
                                     EngineContext& engine_context)
    : ui_ctx_(ui_context), engine_ctx_(engine_context) {}

bool ModelLoaderWidget::load_model_from_path(const std::string& path) {
  spdlog::info("Loading model from: {}", path);

  bool success = false;
  if (path.ends_with(".off")) {
    success = igl::readOFF(path, engine_ctx_.V, engine_ctx_.F, 
                         engine_ctx_.V_normals, engine_ctx_.UV_coords);
  } else if (path.ends_with(".obj")) {
    Eigen::MatrixX3f N;
    success = igl::readOBJ(path, engine_ctx_.V, engine_ctx_.UV_coords,
                         N, engine_ctx_.F);
    engine_ctx_.V_normals = N;
  } else if (path.ends_with(".ply")) {
    success = igl::readPLY(path, engine_ctx_.V, engine_ctx_.F,
                         engine_ctx_.UV_coords, engine_ctx_.V_normals);
  } else if (path.ends_with(".stl")) {
    success = igl::readSTL(path, engine_ctx_.V, engine_ctx_.F,
                         engine_ctx_.V_normals);
  }

  if (!success || engine_ctx_.V.rows() == 0 || engine_ctx_.F.rows() == 0) {
    spdlog::error("Failed to load model or empty mesh from: {}", path);
    return false;
  }

  // Register new mesh and calculate properties
  ui_ctx_.p_surface =
      py::registerSurfaceMesh("model", engine_ctx_.V, engine_ctx_.F);

  auto min = engine_ctx_.V.colwise().minCoeff();
  auto max = engine_ctx_.V.colwise().maxCoeff();
  ui_ctx_.mesh_diag = (max - min).norm();

  ui_ctx_.selection.clear();
  spdlog::info("Loaded model with {} vertices and {} faces",
               engine_ctx_.V.rows(), engine_ctx_.F.rows());

  return true;
}

EventFlag ModelLoaderWidget::draw() {
  // Only enabled in normal mode
  ImGui::BeginDisabled(ui_ctx_.ui_mode != UIMode::Normal);

  if (ImGui::CollapsingHeader("Model Loader", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Load Model")) {
      // Configure file dialog
      std::vector<std::string> filters = {"All Supported Formats",
                                          "*.obj *.off *.ply *.stl",
                                          "Wavefront OBJ",
                                          "*.obj",
                                          "OFF Files",
                                          "*.off",
                                          "Stanford PLY",
                                          "*.ply",
                                          "STL Files",
                                          "*.stl",
                                          "All Files",
                                          "*"};

      // Create and show file dialog
      auto selection =
          pfd::open_file("Select 3D Model", ".", filters, pfd::opt::none)
              .result();

      if (!selection.empty() && load_model_from_path(selection[0])) {
        ImGui::EndDisabled();
        return MeshChange;
      }
    }
  }

  ImGui::EndDisabled();
  return NoEvent;
}

void ModelLoaderWidget::on_event(EventFlag events) {
  // No action needed for events
}
