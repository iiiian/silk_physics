#pragma once

#include <Eigen/Core>

#include "handle.hpp"
#include "manager.hpp"
#include "mesh.hpp"
#include "obstacle.hpp"
#include "pin_group.hpp"
#include "silk/silk.hpp"
#include "solver_data.hpp"

// return ptr to component, return nullptr if handle is invalid
#define ECS_GET_PTR(registry, entity, name) registry.name.get(entity.name)
// remove component, nothing happens if handle is invalid
#define ECS_REMOVE(registry, entity, name) registry.name.remove(entity.name);

namespace silk {

struct Entity {
  Handle cloth_config;
  Handle collision_config;
  Handle tri_mesh;
  Handle pin_group;
  Handle solver_data;
  Handle obstacle;
};

class Registry {
 public:
  Manager<Entity> entity;
  Manager<ClothConfig> cloth_config;
  Manager<CollisionConfig> collision_config;
  Manager<TriMesh> tri_mesh;
  Manager<PinGroup> pin_group;
  Manager<SolverData> solver_data;
  Manager<Obstacle> obstacle;
};

}  // namespace silk
