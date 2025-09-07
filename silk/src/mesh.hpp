#pragma once

#include <Eigen/Core>
#include <optional>

#include "eigen_alias.hpp"
#include "silk/silk.hpp"

namespace silk {

struct TriMesh {
  RMatrixX3f V;
  RMatrixX2i E;
  RMatrixX3i F;
  float avg_edge_length;
};

std::optional<TriMesh> try_make_tri_mesh(MeshConfig mesh_config);

}  // namespace silk
