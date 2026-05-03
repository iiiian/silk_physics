#include <cuda/buffer>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/obstacle_position.hpp"
#include "backend/cuda/solver/obstacle_solver_utils.cuh"
#include "common/mesh.hpp"

namespace silk::cuda {

void batch_reset_obstacle_simulation(Registry& registry) {
  auto obstacles =
      registry.get_entity_with_components<TriMesh, ObstaclePosition>();
  for (uint32_t e : obstacles) {
    auto mesh = registry.get<TriMesh>(e);
    auto position = registry.get<ObstaclePosition>(e);

    if (mesh && position) {
      position->is_static = false;
      position->is_static_twice = false;
      position->curr_position = mesh->V.reshaped<Eigen::RowMajor>();
    }
  }
}

void prepare_obstacle_simulation(Registry& registry, uint32_t& entity,
                                 CudaRuntime rt) {
  auto& e = entity;

  auto config = registry.get<CollisionConfigPlus>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto position = registry.get<ObstaclePosition>(e);

  // Obstacle entity sanity check.
  assert(config && mesh && position);

  // Ensure collider exists and is up-to-date
  using ObjectCollider = collision::ObjectCollider;
  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    auto new_collider = ObjectCollider::from_obstacle(*config, *mesh, rt);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  }
  assert(collider != nullptr);

  if (config->is_updated) {
    collider->update_collision_config(*config, rt);
    config->is_updated = false;
  }

  if (position->is_static_twice) {
    // No-op.
  } else if (position->is_static) {
    // Static once. Update position.
    auto d_pos = host_eigen_to_device(position->curr_position, rt);
    collider->update_position(d_pos, d_pos, rt);
    position->is_static_twice = true;
  } else {
    // Dynamic. Update position.
    auto d_prev = host_eigen_to_device(position->prev_position, rt);
    auto d_curr = host_eigen_to_device(position->curr_position, rt);
    collider->update_position(d_curr, d_prev, rt);
    std::swap(position->curr_position, position->prev_position);
    position->is_static = true;
  }
}

}  // namespace silk::cuda
