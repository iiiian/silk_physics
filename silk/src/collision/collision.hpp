#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "../bbox.hpp"
#include "../solver.hpp"

namespace silk {

template <typename T>
struct Collider {
  Bbox bbox;
  T data;
};

enum class MeshColliderType { POINT, EDGE, TRIANGLE };

struct MeshColliderData {
  MeshColliderType type;

  int v1;
  int v2;
  int v3;
};

using MeshCollider = Collider<MeshColliderData>;

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
  virtual void update_mesh_colliders(std::vector<MeshCollider>& mesh_colliders,
                                     const Eigen::VectorXf& position) const;
  virtual ObstacleStatus get_obstacle_status() const;
};

class CollisionPipeline {
 private:
  class CollisionPipelineImpl;
  std::unique_ptr<CollisionPipelineImpl> impl_;

 public:
  void add_obstacle(IObstacle* obstacle);
  void remove_obstacle(IObstacle* obstacle);
  PositionConstrain resolve_collision(const Eigen::VectorXf& position);

 private:
  void update_obstacles(const Eigen::VectorXf& position);
};

}  // namespace silk
