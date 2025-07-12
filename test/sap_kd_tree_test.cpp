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
  int face_id;
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
  auto& cloth = objects[0];
  int vnum = cloth.V.rows();
  int fnum = cloth.F.rows();

  using Collider = BboxCollider<SimpleColliderdata>;
  std::vector<Collider> colliders;
  colliders.resize(fnum);
  for (int i = 0; i < fnum; ++i) {
    SimpleColliderdata data;
    data.face_id = i;
    data.v0 = cloth.F(i, 0);
    data.v1 = cloth.F(i, 1);
    data.v2 = cloth.F(i, 2);

    Eigen::Matrix3f m;
    m.row(0) = cloth.V.row(data.v0);
    m.row(1) = cloth.V.row(data.v1);
    m.row(2) = cloth.V.row(data.v2);

    Bbox bbox;
    bbox.min = m.colwise().minCoeff();
    bbox.max = m.colwise().maxCoeff();

    colliders[i] = {bbox, data};
  }

  KDTree<SimpleColliderdata> tree{colliders.data(), fnum};
  tree.update();

  CollisionCache<SimpleColliderdata> cache;
  auto dummy_filter = [](const SimpleColliderdata& a,
                         const SimpleColliderdata& b) -> bool { return true; };

  tree.test_self_collision(dummy_filter, cache);
  spdlog::info("detect {} collisions", cache.size());
  for (auto& c : cache) {
    auto& fa = c.first;
    auto& fb = c.second;
    spdlog::info("pair {} {}", fa.face_id, fb.face_id);
  }
}
