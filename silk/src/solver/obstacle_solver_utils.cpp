#include "obstacle_solver_utils.hpp"

#include <silk/silk.hpp>

#include "../ecs.hpp"
#include "../mesh.hpp"
#include "../object_collider_utils.hpp"
#include "../obstacle_position.hpp"

namespace silk {

void batch_reset_obstacle_simulation(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto mesh = registry.get<TriMesh>(e);
    auto position = registry.get<ObstaclePosition>(e);

    if (mesh && position) {
      position->is_static = false;
      position->is_static_twice = false;
      position->curr_position = mesh->V.reshaped<Eigen::RowMajor>();
    }
  }
}

void prepare_obstacle_simulation(Registry& registry, Entity& entity) {
  auto& e = entity;

  auto config = registry.get<CollisionConfig>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto position = registry.get<ObstaclePosition>(e);

  // Obstacle entity sanity check.
  assert(config && mesh && position);

  // Ensure collider exists and is up-to-date
  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    auto new_collider = make_obstacle_object_collider(e.self, *config, *mesh);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  }
  assert(collider != nullptr);
  update_obstacle_object_collider(*config, *position, *collider);

  // Update obstacle position for future step.
  if (position->is_static) {
    position->is_static_twice = true;
  } else {
    std::swap(position->curr_position, position->prev_position);
    position->is_static = true;
  }
}

}  // namespace silk
