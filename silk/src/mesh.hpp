#pragma once

#include <Eigen/Core>
#include <optional>

#include "silk/silk.hpp"

namespace silk {

using RMatrix3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrix3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrix2i = Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor>;

struct TriMesh {
  RMatrix3f V;
  RMatrix2i E;
  RMatrix3i F;
  float avg_edge_length;
};

std::optional<TriMesh> try_make_tri_mesh(MeshConfig mesh_config);

}  // namespace silk
