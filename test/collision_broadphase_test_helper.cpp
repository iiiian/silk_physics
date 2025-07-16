#include "collision_broadphase_test_helper.hpp"

#include <omp.h>

bool SimpleCollider::operator==(const SimpleCollider& other) const {
  return object_id == other.object_id && face_id == other.face_id;
}

namespace std {
std::size_t hash<SimpleCollider>::operator()(const SimpleCollider& k) const {
  return std::hash<int>()(k.face_id);
}
}  // namespace std

void update_colliders(std::vector<SimpleCollider>& colliders,
                      const AlembicObject& object, int object_id,
                      const Eigen::MatrixXf& V) {
  int fnum = object.F.rows();
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

    c.bbox.min = m.colwise().minCoeff();
    c.bbox.max = m.colwise().maxCoeff();
    c.bbox.extend_inplace(h);
  }
}

void brute_force_self_collision(
    std::vector<SimpleCollider>& colliders,
    silk::CollisionFilter<SimpleCollider> filter_callback,
    silk::CollisionCache<SimpleCollider>& cache) {
  std::vector<silk::CollisionCache<SimpleCollider>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders.size(); ++i) {
    for (int j = i + 1; j < colliders.size(); ++j) {
      if (filter_callback(colliders[i], colliders[j])) {
        if (silk::Bbox::is_colliding(colliders[i].bbox, colliders[j].bbox)) {
          thread_local_caches[omp_get_thread_num()].emplace_back(
              colliders.data() + i, colliders.data() + j);
        }
      }
    }
  }
  for (auto& thread_local_cache : thread_local_caches) {
    cache.insert(cache.end(), thread_local_cache.begin(),
                 thread_local_cache.end());
  }
}

void brute_force_group_group_collision(
    std::vector<SimpleCollider>& colliders_a,
    std::vector<SimpleCollider>& colliders_b,
    silk::CollisionFilter<SimpleCollider> filter_callback,
    silk::CollisionCache<SimpleCollider>& cache) {
  std::vector<silk::CollisionCache<SimpleCollider>> thread_local_caches(
      omp_get_max_threads());
#pragma omp parallel for
  for (int i = 0; i < colliders_a.size(); ++i) {
    for (int j = 0; j < colliders_b.size(); ++j) {
      if (filter_callback(colliders_a[i], colliders_b[j])) {
        if (silk::Bbox::is_colliding(colliders_a[i].bbox,
                                     colliders_b[j].bbox)) {
          thread_local_caches[omp_get_thread_num()].emplace_back(
              colliders_a.data() + i, colliders_b.data() + j);
        }
      }
    }
  }
  for (auto& thread_local_cache : thread_local_caches) {
    cache.insert(cache.end(), thread_local_cache.begin(),
                 thread_local_cache.end());
  }
}
