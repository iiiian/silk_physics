#pragma once

#include <Eigen/Core>
#include <optional>

#include "silk/silk.hpp"

namespace silk {

using RMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrixX2i = Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor>;

struct TriMesh {
  RMatrixX3f V;
  RMatrixX2i E;
  RMatrixX3i F;

  float avg_edge_length;
};

std::optional<TriMesh> try_make_tri_mesh(MeshConfig mesh_config);

}  // namespace silk
