#include "backend/cuda/assembly/obstacle_assembler.cuh"

#include <cuda/buffer>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "common/mesh.hpp"
#include "common/pin.hpp"

namespace silk::cuda::assembly {

void assemble_obstacle(ObjRegistry& registry, uint32_t& entity,
                       CudaRuntime rt) {
  auto& e = entity;

  auto config = registry.get<CollisionConfigPlus>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin = registry.get<Pin>(e);

  // Obstacle entity sanity check.
  assert(config && mesh && pin);

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

  if (pin->is_static_twice) {
    // No-op.
  } else if (pin->is_static) {
    // Static once. Update position.
    auto d_pos = host_eigen_to_device(pin->curr_position, rt);
    collider->update_position(d_pos, d_pos, rt);
    pin->is_static_twice = true;
  } else {
    // Dynamic. Update position.
    auto d_prev = host_eigen_to_device(pin->prev_position, rt);
    auto d_curr = host_eigen_to_device(pin->curr_position, rt);
    collider->update_position(d_curr, d_prev, rt);
    std::swap(pin->curr_position, pin->prev_position);
    pin->is_static = true;
  }
}

}  // namespace silk::cuda::assembly
