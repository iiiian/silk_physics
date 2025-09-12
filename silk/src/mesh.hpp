#pragma once

#include "eigen_alias.hpp"

namespace silk {

struct TriMesh {
  // Vertex positions as Nx3 matrix (x, y, z per row).
  RMatrixX3f V;
  // Edge connectivity as Nx2 matrix (vertex indices per row).
  RMatrixX2i E;
  // Face connectivity as Nx3 matrix (vertex indices per row).
  RMatrixX3i F;

  float avg_edge_length;
};

}  // namespace silk
