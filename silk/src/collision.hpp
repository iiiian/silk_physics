#pragma once

#include <Eigen/Core>
#include <vector>

#include "bbox.hpp"
#include "sap_kd_tree.hpp"

namespace silk {

enum class ColliderType { Point, Edge, Triangle };

// mesh level collider
struct Collider {
  Bbox bbox;
  ColliderType type;
  int index;
  // vertex index
  int v1;
  int v2;
  int v3;
  // vertex weight
  float w1;
  float w2;
  float w3;
  // position of vertex at t0
  Eigen::Vector3f v10;
  Eigen::Vector3f v20;
  Eigen::Vector3f v30;
  // position of vertex at t1
  Eigen::Vector3f v11;
  Eigen::Vector3f v21;
  Eigen::Vector3f v31;
};

struct Obstacle {
  Bbox bbox;
  int group;
  bool is_static;
  bool is_pure_obstacle;
  bool is_self_collision_on;
  std::vector<Collider> mesh_colliders;
  KDTree<Collider> mesh_collider_tree;
};

struct CollisionImpact {
  // impact position
  Eigen::Vector3f v1;
  Eigen::Vector3f v2;
  Eigen::Vector3f v3;
  Eigen::Vector3f v4;
  // impact normal, pointing from b to a
  Eigen::Vector3f normal;
  // time of impact
  float toi;
};

struct Collision {
  Collider ca;
  Collider cb;
  CollisionImpact impact;
};

std::vector<Collision> find_collision(std::vector<Obstacle>& obstacles);

}  // namespace silk
