#pragma once

#include <Eigen/Core>
#include <filesystem>

struct AlembicObject {
  std::string name;
  Eigen::MatrixXf V;
  Eigen::MatrixXi F;
  std::vector<Eigen::MatrixXf> series;
};

// load all IPolyMesh with triangle mesh
std::vector<AlembicObject> load_all_meshes(const std::filesystem::path& path_to_abc);
