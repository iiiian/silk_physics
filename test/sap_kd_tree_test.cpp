#include "sap_kd_tree.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <vector>

#include "abc_file_loader.hpp"

using namespace silk;
namespace fs = std::filesystem;

const fs::path root{PHYSICS_SCENE_ROOT};
const fs::path cloth_sphere_abc = root / "cloth_sphere_collision.abc";

struct SimpleColliderdata {
  int v0;
  int v1;
  int v2;
};

TEST_CASE("sap-kdtree-tests", "[collision broadphase]") {
  auto objects = loadAllMeshes(cloth_sphere_abc);
  for (auto& o : objects) {
    spdlog::info("Object {}: {} verts, {} faces, {} step", o.name, o.V.rows(),
                 o.F.rows(), o.series.size());
  }

  // object 0 is the cloth
  auto& c = objects[0];
  int vnum = c.V.rows();
  int fnum = c.F.rows();

  using Collider = BboxCollider<SimpleColliderdata>;
  std::vector<Collider> colliders;
  colliders.resize(fnum);
  for (int i = 0; i < fnum; ++i) {
    SimpleColliderdata data;
    data.v0 = c.F(i, 0);
    data.v1 = c.F(i, 1);
    data.v2 = c.F(i, 2);

    Eigen::Matrix3f m;
    m.row(0) = c.V.row(data.v0);
    m.row(1) = c.V.row(data.v1);
    m.row(2) = c.V.row(data.v2);

    Bbox bbox;
    bbox.min = m.colwise().minCoeff();
    bbox.max = m.colwise().maxCoeff();

    colliders[i] = {bbox, data};
  }

  KDTree<SimpleColliderdata> tree{colliders.data(), fnum};
  tree.update();
}
