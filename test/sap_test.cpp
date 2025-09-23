#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <vector>

#include "abc_file_loader.hpp"
#include "collision_broadphase.hpp"
#include "collision_broadphase_test_utils.hpp"

using namespace silk;
namespace fs = std::filesystem;

const fs::path root{PHYSICS_SCENE_ROOT};
const fs::path cloth_sphere_abc = root / "cloth_sphere_collision.abc";

TEST_CASE("sap-test", "[collision]") {
  spdlog::set_level(spdlog::level::debug);

  auto objects = load_all_meshes(cloth_sphere_abc);

  auto& cloth = objects[0];
  auto& sphere = objects[1];

  int cloth_fnum = cloth.F.rows();
  int sphere_fnum = sphere.F.rows();

  std::vector<SimpleCollider> cloth_colliders(cloth_fnum);
  std::vector<SimpleCollider> sphere_colliders(sphere_fnum);

  std::vector<int> cloth_proxies(cloth_fnum);
  std::vector<int> sphere_proxies(sphere_fnum);

  for (int i = 0; i < cloth_fnum; ++i) {
    cloth_proxies[i] = i;
  }
  for (int i = 0; i < sphere_fnum; ++i) {
    sphere_proxies[i] = i;
  }

  CollisionFilter<SimpleCollider> self_collision_filter =
      [](const SimpleCollider& a, const SimpleCollider& b) -> bool {
    return (a.v0 != b.v0 && a.v0 != b.v1 && a.v0 != b.v2 && a.v1 != b.v0 &&
            a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v0 && a.v2 != b.v1 &&
            a.v2 != b.v2);
  };

  CollisionFilter<SimpleCollider> inter_collision_filter =
      [](const SimpleCollider& a, const SimpleCollider& b) -> bool {
    return true;
  };

  int frame_num = 60;
  for (int i = 0; i < frame_num; ++i) {
    update_colliders(cloth_colliders, cloth, 0, cloth.series[i]);
    update_colliders(sphere_colliders, sphere, 1, sphere.series[i]);

    // self collision
    int axis =
        sap_optimal_axis(cloth_colliders, cloth_proxies.data(), cloth_fnum);
    sap_sort_proxies(cloth_colliders, cloth_proxies.data(), cloth_fnum, axis);
    CollisionCache<SimpleCollider> self_collision_cache;
    sap_sorted_group_self_collision(cloth_colliders, cloth_proxies.data(),
                                    cloth_fnum, axis, self_collision_filter,
                                    self_collision_cache);

    CollisionCache<SimpleCollider> bf_self_collision_cache;
    brute_force_self_collision(cloth_colliders, self_collision_filter,
                               bf_self_collision_cache);

    // inter-collision
    axis =
        sap_optimal_axis(cloth_colliders, cloth_proxies.data(), cloth_fnum,
                         sphere_colliders, sphere_proxies.data(), sphere_fnum);
    sap_sort_proxies(cloth_colliders, cloth_proxies.data(), cloth_fnum, axis);
    sap_sort_proxies(sphere_colliders, sphere_proxies.data(), sphere_fnum,
                     axis);
    CollisionCache<SimpleCollider> inter_collision_cache;
    sap_sorted_group_group_collision(
        cloth_colliders, cloth_proxies.data(), cloth_fnum, sphere_colliders,
        sphere_proxies.data(), sphere_fnum, axis, inter_collision_filter,
        inter_collision_cache);

    CollisionCache<SimpleCollider> bf_inter_collision_cache;
    brute_force_group_group_collision(cloth_colliders, sphere_colliders,
                                      inter_collision_filter,
                                      bf_inter_collision_cache);

    spdlog::info(
        "frame {}: self collision sap/bf {}/{}, inter collision sap/bf {}/{}",
        i, self_collision_cache.size(), bf_self_collision_cache.size(),
        inter_collision_cache.size(), bf_inter_collision_cache.size());

    CHECK(self_collision_cache.size() == bf_self_collision_cache.size());
    CHECK(inter_collision_cache.size() == bf_inter_collision_cache.size());
  }
}
