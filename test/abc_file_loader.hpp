#pragma once

#include <Eigen/Core>

struct AlembicObject {
  std::string name;
  Eigen::MatrixXf V;
  Eigen::MatrixXi F;
  std::vector<Eigen::MatrixXf> series;
};

// load all IPolyMesh with triangle mesh
std::vector<AlembicObject> loadAllMeshes(const std::string& path_to_abc);
