#include "gui_utils.hpp"

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>

std::optional<Mesh> load_mesh_from_file(const std::string& path) {
  std::filesystem::path p{path};

  bool success = false;
  Mesh m;
  if (p.extension() == ".off") {
    success = igl::readOFF(path, m.verts, m.faces);
  } else if (p.extension() == ".obj") {
    success = igl::readOBJ(path, m.verts, m.faces);
  } else if (p.extension() == ".ply") {
    success = igl::readPLY(path, m.verts, m.faces);
  } else if (p.extension() == ".stl") {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      spdlog::error("Failed to open STL file: {}", path);
    } else {
      Eigen::MatrixX3f N;
      success = igl::readSTL(file, m.verts, m.faces, N);
      file.close();
    }
  }

  if (!success || m.verts.rows() == 0 || m.faces.rows() == 0) {
    spdlog::error("Failed to load model or empty mesh from: {}", path);
    return std::nullopt;
  }

  return m;
}
