#include "scene_widget.hpp"

#include <igl/edges.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include <polyscope/polyscope.h>
#include <portable-file-dialogs.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>

namespace py = polyscope;

SceneWidget::SceneWidget(Context& context) : ctx_(context) {}

bool SceneWidget::load_object_from_path(const std::string& path) {
  SPDLOG_INFO("Loading model from: {}", path);

  std::filesystem::path p{path};

  bool success = false;
  Eigen::MatrixXf V;
  Eigen::MatrixXi F;
  Eigen::MatrixX3f N;
  if (p.extension() == ".off") {
    success = igl::readOFF(path, V, F);
  } else if (p.extension() == ".obj") {
    success = igl::readOBJ(path, V, F);
  } else if (p.extension() == ".ply") {
    success = igl::readPLY(path, V, F);
  } else if (p.extension() == ".stl") {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      SPDLOG_ERROR("Failed to open STL file: {}", path);
    } else {
      success = igl::readSTL(file, V, F, N);
      file.close();
    }
  }

  if (!success || V.rows() == 0 || F.rows() == 0) {
    SPDLOG_ERROR("Failed to load model or empty mesh from: {}", path);
    return false;
  }

  SPDLOG_INFO("Loaded model with {} vertices and {} faces", V.rows(), F.rows());

  Eigen::MatrixXi E;
  igl::edges(F, E);

  // Register new mesh
  Object obj;
  obj.name = p.stem();
  obj.mesh = py::registerSurfaceMesh(p.stem(), V, F);
  obj.V = std::move(V);
  obj.F = std::move(F);
  obj.vert_num = V.rows();
  obj.edge_num = E.rows();
  obj.face_num = F.rows();

  ctx_.objects.push_back(std::move(obj));
  ctx_.selection = ctx_.objects.size() - 1;
  py::view::resetCameraToHomeView();

  return true;
}

void SceneWidget::draw() {
  ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
  if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
    // add object botton
    if (ImGui::Button("Load Object")) {
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
      auto file_names =
          pfd::open_file("Select 3D Model", ".", filters, pfd::opt::none)
              .result();

      if (!file_names.empty()) {
        load_object_from_path(file_names[0]);
      }
    }

    ImGui::SameLine();

    ImGui::BeginDisabled((ctx_.selection == -1));
    if (ImGui::Button("Remove Object")) {
      if (ctx_.selection != -1) {
        Object& obj = ctx_.objects[ctx_.selection];

        // remove from polyscope
        py::removeStructure(obj.mesh);

        // remove from silk
        if (obj.silk_handle != 0) {
          switch (obj.type) {
            case SilkObjectType::None:
              assert(false && "has none type but silk handle isn't empty");
              break;
            case SilkObjectType::Cloth: {
              auto res = ctx_.silk_world.remove_cloth(obj.silk_handle);
              assert((res == silk::Result::Success));
              break;
            }
            case SilkObjectType::Obstacle: {
              auto res = ctx_.silk_world.remove_obstacle(obj.silk_handle);
              assert((res == silk::Result::Success));
              break;
            }
          }
        }

        // remove from context
        ctx_.objects.erase(ctx_.objects.begin() + ctx_.selection);
        ctx_.selection = -1;
      }
    }
    ImGui::EndDisabled();

    std::vector<const char*> names;
    names.reserve(ctx_.objects.size());
    for (const auto& obj : ctx_.objects) {
      names.push_back(obj.name.c_str());
    }

    ImGui::SetNextItemWidth(-1);
    ImGui::ListBox("##Objects", &ctx_.selection, names.data(), names.size(), 5);
  }

  ImGui::EndDisabled();
}
