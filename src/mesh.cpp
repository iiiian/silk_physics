#include "mesh.hpp"

#include <Eigen/Core>
#include <cstdint>

#include "common_types.hpp"

Mesh::Mesh(const RMatrixX3f& V, const RMatrixX3i& F) : V(V), F(F) {}

Mesh::Mesh(float* vertices, uint32_t vert_num, int* faces, uint32_t face_num) {
  Eigen::Map<const RMatrixX3f> V_map{vertices, vert_num, 3};
  V = V_map;
  Eigen::Map<const RMatrixX3i> F_map{faces, face_num, 3};
  F = F_map;
}

bool Mesh::is_valid() const {
  uint32_t vnum = V.rows();
  uint32_t fnum = F.rows();

  // empty mesh
  if (vnum == 0 || fnum == 0) {
    return false;
  }

  for (auto face : F.rowwise()) {
    // vertex index out of range
    if (face(0) >= vnum || face(1) >= vnum || face(2) >= vnum) {
      return false;
    }

    // triangle with repeated vertices
    if (face(0) == face(1) || face(1) == face(2) || face(2) == face(0)) {
      return false;
    }
  }

  return true;
}
