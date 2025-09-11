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
#include <string>
#include <vector>

#include "../objects/cloth.hpp"
#include "../objects/obstacle.hpp"

namespace py = polyscope;

SceneWidget::SceneWidget(Context& context) : ctx_(context) {}

// Generate unique name by appending (1), (2), etc. if name already exists
std::string SceneWidget::generate_unique_name(const std::string& base_name) {
  std::string candidate = base_name;
  int counter = 1;
  
  while (true) {
    bool name_exists = false;
    for (const auto& obj : ctx_.objects) {
      if (obj->get_name() == candidate) {
        name_exists = true;
        break;
      }
    }
    
    if (!name_exists) {
      return candidate;
    }
    
    candidate = base_name + " (" + std::to_string(counter) + ")";
    counter++;
  }
}

// Read mesh from path into V,F; returns true on success
bool SceneWidget::load_object_from_path(const std::string& path,
                                        SilkObjectType type) {
  spdlog::info("Loading model from: {}", path);

  std::filesystem::path p{path};

  bool success = false;
  Vert V;
  Face F;
  if (p.extension() == ".off") {
    success = igl::readOFF(path, V, F);
  } else if (p.extension() == ".obj") {
    success = igl::readOBJ(path, V, F);
  } else if (p.extension() == ".ply") {
    success = igl::readPLY(path, V, F);
  } else if (p.extension() == ".stl") {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      spdlog::error("Failed to open STL file: {}", path);
    } else {
      Eigen::MatrixX3f N;
      success = igl::readSTL(file, V, F, N);
      file.close();
    }
  }

  if (!success || V.rows() == 0 || F.rows() == 0) {
    spdlog::error("Failed to load model or empty mesh from: {}", path);
    return false;
  }

  spdlog::info("Loaded model with {} vertices and {} faces", V.rows(),
               F.rows());

  // Auto-center the mesh by computing vertex mean and shifting to origin
  Eigen::Vector3f center = V.colwise().mean();
  V.rowwise() -= center.transpose();
  spdlog::info("Mesh centered by shifting ({}, {}, {})", center.x(), center.y(),
               center.z());

  // Generate unique name for the object
  std::string unique_name = generate_unique_name(p.stem().string());

  bool created = false;
  switch (type) {
    case SilkObjectType::Cloth: {
      auto cloth =
          Cloth::try_make_cloth(&ctx_.silk_world, unique_name, V, F);
      if (cloth) {
        ctx_.objects.push_back(std::make_unique<Cloth>(std::move(*cloth)));
        created = true;
      } else {
        spdlog::error("Failed to create Cloth form {}", path);
      }
      break;
    }
    case SilkObjectType::Obstacle: {
      auto obstacle = Obstacle::try_make_obstacle(&ctx_.silk_world,
                                                  unique_name, V, F);
      if (obstacle) {
        ctx_.objects.push_back(
            std::make_unique<Obstacle>(std::move(*obstacle)));
        created = true;
      } else {
        spdlog::error("Failed to create Obstacle form {}", path);
      }
      break;
    }
  }

  if (created) {
    ctx_.selection = ctx_.objects.size() - 1;
    py::view::resetCameraToHomeView();
  }

  return created;
}

bool SceneWidget::load_object(SilkObjectType type) {
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

  std::vector<std::string> paths =
      pfd::open_file("Select Cloth Mesh", ".", filters, pfd::opt::none)
          .result();

  if (paths.empty()) {
    return false;
  }

  return load_object_from_path(paths[0], type);
}

void SceneWidget::draw() {
  ImGui::BeginDisabled(ctx_.ui_mode != UIMode::Normal);
  if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
    // load object buttons
    if (ImGui::Button("Load Cloth")) {
      load_object(SilkObjectType::Cloth);
    }

    ImGui::SameLine();
    if (ImGui::Button("Load Obstacle")) {
      load_object(SilkObjectType::Obstacle);
    }

    ImGui::BeginDisabled((ctx_.selection == -1));
    if (ImGui::Button("Remove Object")) {
      if (ctx_.selection != -1) {
        // remove from context
        ctx_.objects.erase(ctx_.objects.begin() + ctx_.selection);
        ctx_.selection = -1;
      }
    }
    ImGui::EndDisabled();

    std::vector<std::string> names;
    names.reserve(ctx_.objects.size());
    for (const auto& obj : ctx_.objects) {
      names.push_back(obj->get_name());
    }

    auto get_name = [](void* data, int idx, const char** out_text) -> bool {
      auto* vec = static_cast<std::vector<std::string>*>(data);
      *out_text = vec->at(idx).c_str();
      return true;
    };

    ImGui::SetNextItemWidth(-1);
    ImGui::ListBox("##Objects", &ctx_.selection, get_name, &names,
                   static_cast<int>(names.size()), 5);
  }

  ImGui::EndDisabled();
}
