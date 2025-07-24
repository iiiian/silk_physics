#include "update_object_collider.hpp"

#include <Eigen/Core>

#include "object_collider.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver_data.hpp"

namespace silk {

void update_all_obstacles(Registry& registry,
                          const Eigen::VectorXf& solver_state,
                          const Eigen::VectorXf& prev_solver_state) {
  for (Entity& e : registry.get_all_entities()) {
    auto collision_config = registry.get<CollisionConfig>(e);
    auto pin = registry.get<Pin>(e);
    auto solver_data = registry.get<SolverData>(e);
    auto object_collider = registry.get<ObjectCollider>(e);

    if (collision_config && solver_data && object_collider) {
      update_obstacle(*collision_config, *solver_data, solver_state,
                      prev_solver_state, *object_collider);
      continue;
    }

    if (collision_config && pin && object_collider) {
      update_obstacle(*collision_config, *pin, *object_collider);
      continue;
    }
  }
}

}  // namespace silk
