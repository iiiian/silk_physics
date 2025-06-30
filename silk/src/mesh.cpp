#include <Eigen/Core>
#include <silk/silk.hpp>

namespace silk {

Mesh::Mesh(const Verts& verts, const Faces& faces) : V(verts), F(faces) {}

Mesh::Mesh(float* verts, int vert_num, int* faces, int face_num) {
  Eigen::Map<const Verts> V_map{verts, vert_num, 3};
  V = V_map;
  Eigen::Map<const Faces> F_map{faces, face_num, 3};
  F = F_map;
}

bool Mesh::is_valid() const {
  int vert_num = V.rows();
  int face_num = F.rows();

  // empty mesh
  if (vert_num == 0 || face_num == 0) {
    return false;
  }

  for (auto vert_idx : F.rowwise()) {
    // vertex index out of range
    if (vert_idx(0) >= vert_num || vert_idx(1) >= vert_num ||
        vert_idx(2) >= vert_num) {
      return false;
    }

    // triangle with repeated vertices
    if (vert_idx(0) == vert_idx(1) || vert_idx(1) == vert_idx(2) ||
        vert_idx(2) == vert_idx(0)) {
      return false;
    }
  }

  return true;
}

}  // namespace silk
