#include "backend/cuda/collision/oibvh.cuh"

#include <spdlog/spdlog.h>

#include <catch2/catch_test_macros.hpp>
#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/stream>
#include <filesystem>
#include <vector>

#include "abc_file_loader.hpp"
#include "backend/cuda/cuda_utils.cuh"

using namespace silk::cuda;
namespace fs = std::filesystem;

const fs::path root{PHYSICS_SCENE_ROOT};
const fs::path cloth_sphere_abc = root / "cloth_sphere_collision.abc";

struct SimpleCudaCollider {
  Bbox bbox;
  int object_id;
  int face_id;
  int v0;
  int v1;
  int v2;
};

struct SelfCollisionFilter {
  __both__ bool operator()(const SimpleCudaCollider& a,
                           const SimpleCudaCollider& b) const {
    return (a.v0 != b.v0 && a.v0 != b.v1 && a.v0 != b.v2 && a.v1 != b.v0 &&
            a.v1 != b.v1 && a.v1 != b.v2 && a.v2 != b.v0 && a.v2 != b.v1 &&
            a.v2 != b.v2);
  }
};

struct InterCollisionFilter {
  __both__ bool operator()(const SimpleCudaCollider&,
                           const SimpleCudaCollider&) const {
    return true;
  }
};

std::vector<SimpleCudaCollider> make_colliders(const AlembicObject& object,
                                               int object_id,
                                               const Eigen::MatrixXf& V) {
  int fnum = object.F.rows();
  std::vector<SimpleCudaCollider> colliders(fnum);
  for (int i = 0; i < fnum; ++i) {
    auto& c = colliders[i];
    c.object_id = object_id;
    c.face_id = i;
    c.v0 = object.F(i, 0);
    c.v1 = object.F(i, 1);
    c.v2 = object.F(i, 2);

    Eigen::Matrix3f m;
    m.row(0) = V.row(c.v0);
    m.row(1) = V.row(c.v1);
    m.row(2) = V.row(c.v2);

    float e01_len = (m.row(1) - m.row(0)).norm();
    float e12_len = (m.row(2) - m.row(1)).norm();
    float e20_len = (m.row(0) - m.row(2)).norm();
    float min_len = std::min(e01_len, std::min(e12_len, e20_len));
    float h = 0.05f * min_len;

    c.bbox.min = Vec3f::vec_like(m.colwise().minCoeff());
    c.bbox.max = Vec3f::vec_like(m.colwise().maxCoeff());
    c.bbox = Bbox::pad(c.bbox, h);
  }
  return colliders;
}

Bbox compute_root_bbox(const std::vector<SimpleCudaCollider>& colliders) {
  REQUIRE(!colliders.empty());
  Bbox bbox = colliders[0].bbox;
  for (int i = 1; i < colliders.size(); ++i) {
    bbox = Bbox::merge(bbox, colliders[i].bbox);
  }
  return bbox;
}

template <typename Filter>
int brute_force_self_collision(std::vector<SimpleCudaCollider>& colliders,
                               Filter filter) {
  int count = 0;
  for (int i = 0; i < colliders.size(); ++i) {
    for (int j = i + 1; j < colliders.size(); ++j) {
      if (filter(colliders[i], colliders[j]) &&
          Bbox::is_colliding(colliders[i].bbox, colliders[j].bbox)) {
        ++count;
      }
    }
  }
  return count;
}

template <typename Filter>
int brute_force_group_group_collision(
    std::vector<SimpleCudaCollider>& colliders_a,
    std::vector<SimpleCudaCollider>& colliders_b, Filter filter) {
  int count = 0;
  for (int i = 0; i < colliders_a.size(); ++i) {
    for (int j = 0; j < colliders_b.size(); ++j) {
      if (filter(colliders_a[i], colliders_b[j]) &&
          Bbox::is_colliding(colliders_a[i].bbox, colliders_b[j].bbox)) {
        ++count;
      }
    }
  }
  return count;
}

template <typename Filter>
int oibvh_self_collision_count(OIBVHTree<SimpleCudaCollider>& tree,
                               Filter filter, CudaRuntime rt) {
  auto cache =
      alloc<CollisionPair<SimpleCudaCollider, SimpleCudaCollider>>(rt, 10000);
  int fill = 0;
  tree.test_self_collision(filter, cache, fill, rt);
  rt.stream.sync();
  return fill;
}

template <typename Filter>
int oibvh_group_group_collision_count(
    OIBVHTree<SimpleCudaCollider>& tree,
    ctd::span<const SimpleCudaCollider> colliders, Filter filter,
    CudaRuntime rt) {
  auto cache =
      alloc<CollisionPair<SimpleCudaCollider, SimpleCudaCollider>>(rt, 10000);
  int fill = 0;
  tree.test_ext_collision(colliders, filter, cache, fill, rt);
  rt.stream.sync();
  return fill;
}

TEST_CASE("oibvh-test", "[collision][cuda]") {
  spdlog::set_level(spdlog::level::debug);

  auto objects = load_all_meshes(cloth_sphere_abc);
  auto& cloth = objects[0];
  auto& sphere = objects[1];

  cu::device_ref device = cu::devices[0];
  cu::stream stream{device};
  cu::device_memory_pool mr{device};
  CudaRuntime rt{.stream = stream, .mr = mr.as_ref()};

  auto cloth_colliders = make_colliders(cloth, 0, cloth.series[0]);
  auto sphere_colliders = make_colliders(sphere, 1, sphere.series[0]);

  auto d_cloth_colliders =
      vec_like_to_device<SimpleCudaCollider>(cloth_colliders, rt);
  auto d_sphere_colliders =
      vec_like_to_device<SimpleCudaCollider>(sphere_colliders, rt);

  OIBVHTree<SimpleCudaCollider> cloth_tree{compute_root_bbox(cloth_colliders),
                                           std::move(d_cloth_colliders), rt};
  OIBVHTree<SimpleCudaCollider> sphere_tree{compute_root_bbox(sphere_colliders),
                                            std::move(d_sphere_colliders), rt};

  int frame_num = 60;
  for (int frame = 0; frame < frame_num; ++frame) {
    cloth_colliders = make_colliders(cloth, 0, cloth.series[frame]);
    sphere_colliders = make_colliders(sphere, 1, sphere.series[0]);

    cu::copy_bytes(rt.stream, cloth_colliders, cloth_tree.get_colliders());
    cu::copy_bytes(rt.stream, sphere_colliders, sphere_tree.get_colliders());
    cloth_tree.update(compute_root_bbox(cloth_colliders), rt);
    sphere_tree.update(compute_root_bbox(sphere_colliders), rt);

    int self_count =
        oibvh_self_collision_count(cloth_tree, SelfCollisionFilter{}, rt);
    int bf_self_count =
        brute_force_self_collision(cloth_colliders, SelfCollisionFilter{});

    int inter_count = oibvh_group_group_collision_count(
        cloth_tree, sphere_tree.get_colliders(), InterCollisionFilter{}, rt);
    int bf_inter_count = brute_force_group_group_collision(
        cloth_colliders, sphere_colliders, InterCollisionFilter{});

    spdlog::info(
        "frame {}: self collision oibvh/bf {}/{}, inter collision oibvh/bf "
        "{}/{}",
        frame, self_count, bf_self_count, inter_count, bf_inter_count);

    CHECK(self_count == bf_self_count);
    CHECK(inter_count == bf_inter_count);
  }
}
