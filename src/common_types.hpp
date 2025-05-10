#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_set>

using Matrix69f = Eigen::Matrix<float, 6, 9>;
using RMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajorBit>;
using RMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajorBit>;

struct Segment {
  std::string name;
  long id;
  std::unordered_set<int> verts;
  // x, y, z displacement
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  // α, β, γ rotation angles in radians
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
};
