#include <omp.h>
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
  int object_id;
  int face_id;
  int v0;
  int v1;
  int v2;

  bool operator==(const SimpleColliderdata& other) const {
    return object_id == other.object_id && face_id == other.face_id;
  }
};

template <>
struct std::hash<SimpleColliderdata> {
  std::size_t operator()(const SimpleColliderdata& k) const {
    return std::hash<int>()(k.face_id);
  }
};

void update_colliders(std::vector<BboxCollider<SimpleColliderdata>>& colliders,
                      const AlembicObject& object, int object_id,
                      const Eigen::MatrixXf& V) {
  int fnum = object.F.rows();
  for (int i = 0; i < fnum; ++i) {
    SimpleColliderdata data;
    data.object_id = object_id;
    data.face_id = i;
    data.v0 = object.F(i, 0);
    data.v1 = object.F(i, 1);
    data.v2 = object.F(i, 2);

    Eigen::Matrix3f m;
    m.row(0) = V.row(data.v0);
    m.row(1) = V.row(data.v1);
    m.row(2) = V.row(data.v2);

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
}

void brute_force_self_collision(
    const std::vector<BboxCollider<SimpleColliderdata>>& colliders,
    CollisionFilterCallback<SimpleColliderdata> filter_callback,
    CollisionCache<SimpleColliderdata>& cache) {
  std::vector<CollisionCache<SimpleColliderdata>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders.size(); ++i) {
    for (int j = i + 1; j < colliders.size(); ++j) {
      if (filter_callback(colliders[i].data, colliders[j].data)) {
        if (Bbox::is_colliding(colliders[i].bbox, colliders[j].bbox)) {
          thread_local_caches[omp_get_thread_num()].emplace_back(
              colliders[i].data, colliders[j].data);
        }
      }
    }
  }
  for (const auto& thread_local_cache : thread_local_caches) {
    cache.insert(cache.end(), thread_local_cache.begin(),
                 thread_local_cache.end());
  }
}

void brute_force_group_group_collision(
    const std::vector<BboxCollider<SimpleColliderdata>>& colliders_a,
    const std::vector<BboxCollider<SimpleColliderdata>>& colliders_b,
    CollisionFilterCallback<SimpleColliderdata> filter_callback,
    CollisionCache<SimpleColliderdata>& cache) {
  std::vector<CollisionCache<SimpleColliderdata>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders_a.size(); ++i) {
    for (int j = 0; j < colliders_b.size(); ++j) {
      if (filter_callback(colliders_a[i].data, colliders_b[j].data)) {
        if (Bbox::is_colliding(colliders_a[i].bbox, colliders_b[j].bbox)) {
          thread_local_caches[omp_get_thread_num()].emplace_back(
              colliders_a[i].data, colliders_b[j].data);
        }
      }
    }
  }
  for (const auto& thread_local_cache : thread_local_caches) {
    cache.insert(cache.end(), thread_local_cache.begin(),
                 thread_local_cache.end());
  }
}

TEST_CASE("sap-animation-test", "[collision broadphase]") {
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

  int frame_num = cloth.series.size();
  for (int i = 0; i < frame_num; ++i) {
    spdlog::info("Testing frame {}", i);

    update_colliders(cloth_colliders, cloth, 0, cloth.series[i]);
    update_colliders(sphere_colliders, sphere, 1, sphere.series[i]);

    spdlog::info("sap self collision test");
    // self collision
    int axis = sap_optimal_axis(cloth_proxies.data(), cloth_fnum);
    sap_sort_proxies(cloth_proxies.data(), cloth_fnum, axis);
    CollisionCache<SimpleColliderdata> self_collision_cache;
    sap_sorted_group_self_collision(cloth_proxies.data(), cloth_fnum, axis,
                                    self_collision_filter,
                                    self_collision_cache);

    spdlog::info("brute force self collision test");
    CollisionCache<SimpleColliderdata> bf_self_collision_cache;
    brute_force_self_collision(cloth_colliders, self_collision_filter,
                               bf_self_collision_cache);

    spdlog::info("sap group collision test");
    // inter-collision
    axis = sap_optimal_axis(cloth_proxies.data(), cloth_fnum,
                            sphere_proxies.data(), sphere_fnum);
    sap_sort_proxies(cloth_proxies.data(), cloth_fnum, axis);
    sap_sort_proxies(sphere_proxies.data(), sphere_fnum, axis);
    CollisionCache<SimpleColliderdata> inter_collision_cache;
    sap_sorted_group_group_collision(
        cloth_proxies.data(), cloth_fnum, sphere_proxies.data(), sphere_fnum,
        axis, inter_collision_filter, inter_collision_cache);

    spdlog::info("brute force group collision test");
    CollisionCache<SimpleColliderdata> bf_inter_collision_cache;
    brute_force_group_group_collision(cloth_colliders, sphere_colliders,
                                      inter_collision_filter,
                                      bf_inter_collision_cache);

    spdlog::info(
        "frame {}: self collision sap/bf {}/{}, inter collision sap/bf {}/{}"
        "\n",
        i, self_collision_cache.size(), bf_self_collision_cache.size(),
        inter_collision_cache.size(), bf_inter_collision_cache.size());

    CHECK(self_collision_cache.size() == bf_self_collision_cache.size());
    CHECK(inter_collision_cache.size() == bf_inter_collision_cache.size());
  }
}
