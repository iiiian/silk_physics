#include "mesh.hpp"

#include <igl/edges.h>

#include <Eigen/Core>
#include <cassert>

namespace silk {

std::optional<TriMesh> try_make_tri_mesh(MeshConfig mesh_config,
                                         PinConfig pinning_config) {
  assert((mesh_config.validate() == Result::Success &&
          pinning_config.validate() == Result::Success));

  auto& mc = mesh_config;
  auto& pc = pinning_config;

  TriMesh mesh;
  mesh.V = Eigen::Map<RMatrixX3f>(mc.vertices, mc.vert_num, 3);
  mesh.F = Eigen::Map<RMatrixX3i>(mc.faces, mc.face_num, 3);
  igl::edges(mesh.F, mesh.E);

  if (pc.pinned_num != 0) {
    mesh.pinned = Eigen::Map<Eigen::VectorXi>(pc.pinned_index, pc.pinned_num);
  }

  // check vertex index of pinned vertices lies in range
  int max = mesh.pinned.maxCoeff();
  if (max >= mesh_config.vert_num) {
    return std::nullopt;
  }

  return mesh;
}

}  // namespace silk
