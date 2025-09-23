#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <vector>

#include "abc_file_loader.hpp"
#include "collision_broadphase.hpp"
#include "collision_broadphase_test_utils.hpp"

using namespace silk;
namespace fs = std::filesystem;

const fs::path root{PHYSICS_SCENE_ROOT};
const fs::path cloth_sphere_abc = root / "cloth_sphere_collision_dense.abc";

TEST_CASE("sap-kd-tree-benchmark", "[collision]") {
  // object 0 is the cloth
  // object 1 is a icosphere
  // the cloth will fall onto the icosphere
  auto objects = load_all_meshes(cloth_sphere_abc);

  auto& cloth = objects[0];
  auto& sphere = objects[1];

  int cloth_fnum = cloth.F.rows();
  int sphere_fnum = sphere.F.rows();

  KDTree<SimpleCollider> cloth_tree;
  cloth_tree.init(std::vector<SimpleCollider>(cloth_fnum));
  KDTree<SimpleCollider> sphere_tree;
  sphere_tree.init(std::vector<SimpleCollider>(sphere_fnum));

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

  auto& sphere_colliders = sphere_tree.get_colliders();
  auto& cloth_colliders = cloth_tree.get_colliders();

  // sphere is static, only update once.
  update_colliders(sphere_colliders, sphere, 1, sphere.series[0]);
  silk::Bbox bbox = sphere_colliders[0].bbox;
  for (int i = 0; i < sphere_colliders.size(); ++i) {
    bbox.merge_inplace(sphere_colliders[i].bbox);
  }
  sphere_tree.update(bbox);

  namespace ch = std::chrono;
  ch::nanoseconds elapsed{0};
  auto t0 = ch::steady_clock::now();

  int frame_num = 60;
  for (int i = 0; i < frame_num; ++i) {
    update_colliders(cloth_colliders, cloth, 0, cloth.series[i]);
    silk::Bbox bbox = cloth_colliders[0].bbox;
    for (int i = 0; i < cloth_colliders.size(); ++i) {
      bbox.merge_inplace(cloth_colliders[i].bbox);
    }
    cloth_tree.update(bbox);

    // self collision
    CollisionCache<SimpleCollider> self_collision_cache;
    cloth_tree.test_self_collision(self_collision_filter, self_collision_cache);

    // inter-collision
    CollisionCache<SimpleCollider> inter_collision_cache;
    KDTree<SimpleCollider>::test_tree_collision(
        cloth_tree, sphere_tree, inter_collision_filter, inter_collision_cache);

    // spdlog::info("frame {}: self collision {}, inter collision {}", i,
    //              self_collision_cache.size(), inter_collision_cache.size());
  }

  elapsed += ch::steady_clock::now() - t0;
  spdlog::info("total {} ms",
               ch::duration_cast<ch::milliseconds>(elapsed).count());
}
