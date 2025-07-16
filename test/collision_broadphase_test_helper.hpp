#pragma once

#include <Eigen/Core>
#include <vector>

#include "../bbox.hpp"
#include "abc_file_loader.hpp"
#include "collision_internal.hpp"

struct SimpleCollider {
  silk::Bbox bbox;
  int object_id;
  int face_id;
  int v0;
  int v1;
  int v2;

  bool operator==(const SimpleCollider& other) const;
};

namespace std {
template <>
struct hash<SimpleCollider> {
  std::size_t operator()(const SimpleCollider& k) const;
};
}  // namespace std

void update_colliders(std::vector<SimpleCollider>& colliders,
                      const AlembicObject& object, int object_id,
                      const Eigen::MatrixXf& V);

void brute_force_self_collision(
    const std::vector<SimpleCollider>& colliders,
    silk::CollisionFilter<SimpleCollider> filter_callback,
    silk::CollisionCache<SimpleCollider>& cache);

void brute_force_group_group_collision(
    const std::vector<SimpleCollider>& colliders_a,
    const std::vector<SimpleCollider>& colliders_b,
    silk::CollisionFilter<SimpleCollider> filter_callback,
    silk::CollisionCache<SimpleCollider>& cache);
