#include <Eigen/Core>
#include <cstdint>
#include <silk/silk.hpp>

Mesh::Mesh(const Verts& verts, const Faces& faces) : V(verts), F(faces) {}

Mesh::Mesh(float* verts, uint32_t vert_num, int* faces, uint32_t face_num) {
  Eigen::Map<const Verts> V_map{verts, vert_num, 3};
  V = V_map;
  Eigen::Map<const Faces> F_map{faces, face_num, 3};
  F = F_map;
}

bool Mesh::is_valid() const {
  uint32_t vnum = V.rows();
  uint32_t fnum = F.rows();

  // empty mesh
  if (vnum == 0 || fnum == 0) {
    return false;
  }

  for (auto f : F.rowwise()) {
    // vertex index out of range
    if (f(0) >= vnum || f(1) >= vnum || f(2) >= vnum) {
      return false;
    }

    // triangle with repeated vertices
    if (f(0) == f(1) || f(1) == f(2) || f(2) == f(0)) {
      return false;
    }
  }

  return true;
}
