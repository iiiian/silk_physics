#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "../bbox.hpp"

namespace silk {

enum class MeshColliderType { Point, Edge, Triangle };

struct MeshCollider {
  Bbox bbox;
  MeshColliderType type;
  int index;
  // vertex index
  int v1;
  int v2;
  int v3;
};

struct MeshColliderDetail {
  // position of vertex at t0
  Eigen::Vector3f v10;
  Eigen::Vector3f v20;
  Eigen::Vector3f v30;
  // position of vertex at t1
  Eigen::Vector3f v11;
  Eigen::Vector3f v21;
  Eigen::Vector3f v31;
  // weight (inverse mass)
  float w1;
  float w2;
  float w3;
};

struct MeshColliderImpact {
  // impact position
  Eigen::Vector3f v1;
  Eigen::Vector3f v2;
  Eigen::Vector3f v3;
  Eigen::Vector3f normal;
  int index;
  float toi;
  float weight;
};

struct ObstacleStatus {
  // collision group:
  //  -1  -> disable collision with others
  // >=0  -> collide with those in the same group
  int group;
  bool is_static;
  bool is_pure_obstacle;
  bool is_self_collision_on;
};

class IObstacle {
 public:
  virtual std::vector<MeshCollider> init_mesh_colliders() const;
  virtual void update_mesh_colliders(
      std::vector<MeshCollider>& mesh_colliders) const;
  virtual MeshColliderDetail get_mesh_collider_detail(int index) const = 0;
  virtual void resolve_impact(MeshColliderImpact impact) = 0;
  virtual ObstacleStatus get_obstacle_status() const;
};

enum class CollisionConstrainType {
  Point,
  Edge,
  Triangle,
  PointTriangle,
  EdgeEdge
};

class CollisionPipeline {
 private:
  class CollisionPipelineImpl;
  std::unique_ptr<CollisionPipelineImpl> impl_;

 public:
  void add_obstacle(IObstacle* obstacle);
  void remove_obstacle(IObstacle* obstacle);
  void resolve_collision();
};

}  // namespace silk
