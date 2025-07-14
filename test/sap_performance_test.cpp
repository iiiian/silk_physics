#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <vector>

#include "abc_file_loader.hpp"
#include "collision_broadphase_test_helper.hpp"
#include "sap.hpp"

using namespace silk;
namespace fs = std::filesystem;

const fs::path root{PHYSICS_SCENE_ROOT};
const fs::path cloth_sphere_abc = root / "cloth_sphere_collision_dense.abc";

TEST_CASE("sap-animation-performance-test",
          "[collision broadphase][performance]") {
  auto objects = loadAllMeshes(cloth_sphere_abc);

  auto& cloth = objects[0];
  auto& sphere = objects[1];

  int cloth_fnum = cloth.F.rows();
  int sphere_fnum = sphere.F.rows();

  std::vector<BboxCollider<SimpleColliderdata>> cloth_colliders(cloth_fnum);
  std::vector<BboxCollider<SimpleColliderdata>> sphere_colliders(sphere_fnum);

  std::vector<BboxColliderProxy<SimpleColliderdata>> cloth_proxies(cloth_fnum);
  std::vector<BboxColliderProxy<SimpleColliderdata>> sphere_proxies(
      sphere_fnum);

  for (int i = 0; i < cloth_fnum; ++i) {
    cloth_proxies[i] = cloth_colliders.data() + i;
  }
  for (int i = 0; i < sphere_fnum; ++i) {
    sphere_proxies[i] = sphere_colliders.data() + i;
  }

  CollisionFilterCallback<SimpleColliderdata> self_collision_filter =
      [](const SimpleColliderdata& a, const SimpleColliderdata& b) -> bool {
    return (a.v0 != b.v0 && a.v0 != b.v1 && a.v0 != b.v2 && a.v1 != b.v0 &&
            a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v0 && a.v2 != b.v1 &&
            a.v2 != b.v2);
  };

  CollisionFilterCallback<SimpleColliderdata> inter_collision_filter =
      [](const SimpleColliderdata& a, const SimpleColliderdata& b) -> bool {
    return true;
  };

  int frame_num = 60;
  for (int i = 0; i < frame_num; ++i) {
    // spdlog::info("Testing frame {}", i);

    update_colliders(cloth_colliders, cloth, 0, cloth.series[i]);
    update_colliders(sphere_colliders, sphere, 1, sphere.series[i]);

    // self collision
    int axis = sap_optimal_axis(cloth_proxies.data(), cloth_fnum);
    sap_sort_proxies(cloth_proxies.data(), cloth_fnum, axis);
    CollisionCache<SimpleColliderdata> self_collision_cache;
    sap_sorted_group_self_collision(cloth_proxies.data(), cloth_fnum, axis,
                                    self_collision_filter,
                                    self_collision_cache);

    // inter-collision
    // axis = sap_optimal_axis(cloth_proxies.data(), cloth_fnum,
    //                         sphere_proxies.data(), sphere_fnum);
    // sap_sort_proxies(cloth_proxies.data(), cloth_fnum, axis);
    // sap_sort_proxies(sphere_proxies.data(), sphere_fnum, axis);
    // CollisionCache<SimpleColliderdata> inter_collision_cache;
    // sap_sorted_group_group_collision(
    //     cloth_proxies.data(), cloth_fnum, sphere_proxies.data(), sphere_fnum,
    //     axis, inter_collision_filter, inter_collision_cache);

    // spdlog::info(
    //     "frame {}: self collision {}, inter collision {}",
    //     i, self_collision_cache.size(), inter_collision_cache.size());
  }
}
