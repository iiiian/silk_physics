#pragma once

#include <Eigen/Core>
#include <optional>

#include "silk/silk.hpp"

namespace silk {

struct TriMesh {
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> V;
  Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> E;
  Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor> F;

  float avg_edge_length;
};

std::optional<TriMesh> try_make_tri_mesh(MeshConfig mesh_config);

}  // namespace silk
