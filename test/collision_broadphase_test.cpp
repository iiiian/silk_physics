#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <functional>
#include <vector>

#include "abc_file_loader.hpp"
#include "sap.hpp"

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

TEST_CASE("sap-test", "[collision broadphase]") {
  auto objects = loadAllMeshes(cloth_sphere_abc);

  // object 0 is the cloth
  auto& cloth = objects[0];
  int vnum = cloth.V.rows();
  int fnum = cloth.F.rows();

  std::vector<BboxCollider<SimpleColliderdata>> colliders(fnum);
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

    float e01_len = (m.row(1) - m.row(0)).norm();
    float e12_len = (m.row(2) - m.row(1)).norm();
    float e20_len = (m.row(0) - m.row(2)).norm();

    float min_len = std::min(e01_len, std::min(e12_len, e20_len));
    float h = 0.05f * min_len;

    Bbox bbox;
    bbox.min = m.colwise().minCoeff();
    bbox.max = m.colwise().maxCoeff();
    bbox.extend_inplace(h);

    colliders[i] = {bbox, data};
  }

  std::vector<BboxColliderProxy<SimpleColliderdata>> proxies(fnum);
  for (int i = 0; i < fnum; ++i) {
    proxies[i] = colliders.data() + i;
  }

  int axis = sap_optimal_axis(proxies.data(), fnum);
  sap_sort_proxies(proxies.data(), fnum, axis);

  CollisionCache<SimpleColliderdata> cache;
  CollisionFilterCallback<SimpleColliderdata> filter =
      [](const SimpleColliderdata& a, const SimpleColliderdata& b) -> bool {
    return (a.v0 != b.v0 && a.v0 != b.v1 && a.v0 != b.v2 && a.v1 != b.v0 &&
            a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v0 && a.v2 != b.v1 &&
            a.v2 != b.v2);
  };

  sap_sorted_group_self_collision(proxies.data(), fnum, axis, filter, cache);
  spdlog::info("detect {} collisions", cache.size());
  for (auto& c : cache) {
    auto& fa = c.first;
    auto& fb = c.second;
    spdlog::info("pair {}: {}, {}, {} | {}: {}, {}, {}", fa.face_id, fa.v0,
                 fa.v1, fa.v2, fb.face_id, fb.v0, fb.v1, fb.v2);
  }
}
