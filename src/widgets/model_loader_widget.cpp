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

  // Clear previous attributes before loading
  engine_ctx_.V_normals.resize(0, 3);
  engine_ctx_.UV_coords.resize(0, 2);

  bool success = false;
  if (path.ends_with(".off")) {
    // igl::readOFF typically only reads V and F
    success = igl::readOFF(path, engine_ctx_.V, engine_ctx_.F);
    // OFF doesn't standardly store normals or UVs this way
    engine_ctx_.V_normals.resize(0, 3);
    engine_ctx_.UV_coords.resize(0, 2);
  } else if (path.ends_with(".obj")) {
    // Use the overload that reads V, TC, CN, F, FTC, FN
    eg::MatrixXi FTC, FN; // Temporary matrices for face texture/normal indices
    success = igl::readOBJ(path, engine_ctx_.V, engine_ctx_.UV_coords,
                           engine_ctx_.V_normals, engine_ctx_.F, FTC, FN);
    // If normals or UVs weren't present, clear the matrices
    if (engine_ctx_.V_normals.rows() != engine_ctx_.V.rows()) {
        engine_ctx_.V_normals.resize(0, 3);
        spdlog::warn("OBJ file did not contain vertex normals or format mismatch.");
    }
     if (engine_ctx_.UV_coords.rows() != engine_ctx_.V.rows()) {
        engine_ctx_.UV_coords.resize(0, 2);
        spdlog::warn("OBJ file did not contain texture coordinates or format mismatch.");
    }
  } else if (path.ends_with(".ply")) {
    // igl::readPLY basic version reads V and F
    // Reading additional attributes requires more complex handling
    success = igl::readPLY(path, engine_ctx_.V, engine_ctx_.F);
    engine_ctx_.V_normals.resize(0, 3);
    engine_ctx_.UV_coords.resize(0, 2);
  } else if (path.ends_with(".stl")) {
    // igl::readSTL reads V, F, and N (face normals)
    eg::MatrixX3f N_face; // Temporary matrix for face normals
    success = igl::readSTL(path, engine_ctx_.V, engine_ctx_.F, N_face);
    // STL provides face normals, not vertex normals directly
    engine_ctx_.V_normals.resize(0, 3);
    engine_ctx_.UV_coords.resize(0, 2);
  }

  if (!success || engine_ctx_.V.rows() == 0 || engine_ctx_.F.rows() == 0) {
    spdlog::error("Failed to load model or empty mesh from: {}", path);
    // Clear context if loading failed
    engine_ctx_.V.resize(0,3);
    engine_ctx_.F.resize(0,3);
    engine_ctx_.V_normals.resize(0, 3);
    engine_ctx_.UV_coords.resize(0, 2);
    if (ui_ctx_.p_surface) {
        py::removeStructure("model");
        ui_ctx_.p_surface = nullptr;
    }
    ui_ctx_.mesh_diag = 0;
    ui_ctx_.selection.clear();
    return false;
  }

  // Register new mesh and calculate properties
  // Remove existing mesh if present
   if (ui_ctx_.p_surface) {
        py::removeStructure("model");
   }
  ui_ctx_.p_surface =
      py::registerSurfaceMesh("model", engine_ctx_.V, engine_ctx_.F);

  auto min = engine_ctx_.V.colwise().minCoeff();
  auto max = engine_ctx_.V.colwise().maxCoeff();
  ui_ctx_.mesh_diag = (max - min).norm();

  ui_ctx_.selection.clear();
  spdlog::info("Loaded model with {} vertices and {} faces",
               engine_ctx_.V.rows(), engine_ctx_.F.rows());
  if(engine_ctx_.V_normals.rows() > 0) {
      spdlog::info("  - Vertex normals loaded.");
  }
   if(engine_ctx_.UV_coords.rows() > 0) {
      spdlog::info("  - UV coordinates loaded.");
  }


  return true;
}

EventFlag ModelLoaderWidget::draw() {
  // Only enabled in normal mode
  ImGui::BeginDisabled(ui_ctx_.ui_mode != UIMode::Normal);

  EventFlag generated_event = NoEvent;
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
        generated_event = MeshChange;
      }
    }
  }

  ImGui::EndDisabled();
  // Return MeshChange immediately if a model was loaded, otherwise NoEvent
  return generated_event;
}

void ModelLoaderWidget::on_event(EventFlag events) {
  // No action needed for events
}
