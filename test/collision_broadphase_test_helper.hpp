#pragma once

#include <Eigen/Core>
#include <vector>

#include "abc_file_loader.hpp"
#include "bbox.hpp"
#include "collision_helper.hpp"

struct SimpleColliderdata {
  int object_id;
  int face_id;
  int v0;
  int v1;
  int v2;

  bool operator==(const SimpleColliderdata& other) const;
};

namespace std {
template <>
struct hash<SimpleColliderdata> {
  std::size_t operator()(const SimpleColliderdata& k) const;
};
}  // namespace std

void update_colliders(std::vector<silk::BboxCollider<SimpleColliderdata>>& colliders,
                      const AlembicObject& object, int object_id,
                      const Eigen::MatrixXf& V);

void brute_force_self_collision(
    const std::vector<silk::BboxCollider<SimpleColliderdata>>& colliders,
    silk::CollisionFilterCallback<SimpleColliderdata> filter_callback,
    silk::CollisionCache<SimpleColliderdata>& cache);

void brute_force_group_group_collision(
    const std::vector<silk::BboxCollider<SimpleColliderdata>>& colliders_a,
    const std::vector<silk::BboxCollider<SimpleColliderdata>>& colliders_b,
    silk::CollisionFilterCallback<SimpleColliderdata> filter_callback,
    silk::CollisionCache<SimpleColliderdata>& cache);
