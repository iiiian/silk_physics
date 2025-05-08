#include "model_loader_widget.hpp"

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include <polyscope/polyscope.h>
#include <portable-file-dialogs.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <fstream>

#include "../common_types.hpp"

namespace py = polyscope;
namespace eg = Eigen;

ModelLoaderWidget::ModelLoaderWidget(UIContext& ui_context,
                                     EngineContext& engine_context)
    : ui_ctx_(ui_context), engine_ctx_(engine_context) {}

bool ModelLoaderWidget::load_model_from_path(const std::string& path) {
  spdlog::info("Loading model from: {}", path);

  bool success = false;
  RMatrixX3f V;
  eg::MatrixX3i F;
  eg::MatrixX3f N;
  if (path.ends_with(".off")) {
    success = igl::readOFF(path, V, F);
  } else if (path.ends_with(".obj")) {
    success = igl::readOBJ(path, V, F);
  } else if (path.ends_with(".ply")) {
    success = igl::readPLY(path, V, F);
  } else if (path.ends_with(".stl")) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      spdlog::error("Failed to open STL file: {}", path);
    } else {
      success = igl::readSTL(file, V, F, N);
      file.close();
    }
  }

  if (!success || V.rows() == 0 || F.rows() == 0) {
    spdlog::error("Failed to load model or empty mesh from: {}", path);
    return false;
  }

  // Register new mesh
  auto min = V.colwise().minCoeff();
  auto max = V.colwise().maxCoeff();
  ui_ctx_.mesh_diag = (max - min).norm();
  ui_ctx_.selection.clear();
  ui_ctx_.p_surface = py::registerSurfaceMesh("model", V, F);

  engine_ctx_.V = std::move(V);
  engine_ctx_.F = std::move(F);

  spdlog::info("Loaded model with {} vertices and {} faces",
               engine_ctx_.V.rows(), engine_ctx_.F.rows());

  return true;
}

EventFlag ModelLoaderWidget::draw() {
  // Only enabled in normal mode
  ImGui::BeginDisabled(ui_ctx_.ui_mode != UIMode::Normal);

  EventFlag generated_event = EventFlag::NoEvent;
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
        generated_event = EventFlag::MeshChange;
      }
    }
  }

  ImGui::EndDisabled();
  // Return MeshChange if a model was loaded, otherwise NoEvent
  return generated_event;
}

void ModelLoaderWidget::on_event(EventFlag events) {}
