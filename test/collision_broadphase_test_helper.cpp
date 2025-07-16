#include "collision_broadphase_test_helper.hpp"

#include <omp.h>

bool SimpleColliderdata::operator==(const SimpleColliderdata& other) const {
  return object_id == other.object_id && face_id == other.face_id;
}

namespace std {
std::size_t hash<SimpleColliderdata>::operator()(
    const SimpleColliderdata& k) const {
  return std::hash<int>()(k.face_id);
}
}  // namespace std

void update_colliders(
    std::vector<silk::Collider<SimpleColliderdata>>& colliders,
    const AlembicObject& object, int object_id, const Eigen::MatrixXf& V) {
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

    silk::Bbox bbox;
    bbox.min = m.colwise().minCoeff();
    bbox.max = m.colwise().maxCoeff();
    bbox.extend_inplace(h);

    colliders[i] = {bbox, data};
  }
}

void brute_force_self_collision(
    const std::vector<silk::Collider<SimpleColliderdata>>& colliders,
    silk::CollisionFilterCallback<SimpleColliderdata> filter_callback,
    silk::CollisionCache<SimpleColliderdata>& cache) {
  std::vector<silk::CollisionCache<SimpleColliderdata>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders.size(); ++i) {
    for (int j = i + 1; j < colliders.size(); ++j) {
      if (filter_callback(colliders[i].data, colliders[j].data)) {
        if (silk::Bbox::is_colliding(colliders[i].bbox, colliders[j].bbox)) {
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
    const std::vector<silk::Collider<SimpleColliderdata>>& colliders_a,
    const std::vector<silk::Collider<SimpleColliderdata>>& colliders_b,
    silk::CollisionFilterCallback<SimpleColliderdata> filter_callback,
    silk::CollisionCache<SimpleColliderdata>& cache) {
  std::vector<silk::CollisionCache<SimpleColliderdata>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders_a.size(); ++i) {
    for (int j = 0; j < colliders_b.size(); ++j) {
      if (filter_callback(colliders_a[i].data, colliders_b[j].data)) {
        if (silk::Bbox::is_colliding(colliders_a[i].bbox,
                                     colliders_b[j].bbox)) {
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
