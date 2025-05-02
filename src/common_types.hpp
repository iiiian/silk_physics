#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_set>

struct Segment {
  std::string name;
  long id;
  std::unordered_set<int> verts;
  // x, y, z displacement
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  // α, β, γ rotation angles in radians
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
};
