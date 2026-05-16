#include "backend/cuda/assembly/obstacle_assembler.cuh"

#include <cuda/buffer>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/pin.hpp"
#include "common/initial_state.hpp"
#include "common/mesh.hpp"

namespace silk::cuda {

void assemble_obstacle(ObjRegistry& registry, uint32_t& entity,
                       CudaRuntime rt) {
  auto& e = entity;

  auto config = registry.get<CollisionConfigPlus>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto init_state = registry.get<InitialState>(e);
  auto pin_index = registry.get<PinIndex>(e);

  // Obstacle entity sanity check.
  assert(config && mesh && init_state && pin_index);

  auto pin_pos = registry.get<PinPosition>(e);
  if (!pin_pos) {
    std::span<const float> pos_span(init_state->position.data(),
                                    init_state->position.size());
    registry.set(e, PinPosition{*pin_index, pos_span});
  }

  // Ensure collider exists and is up-to-date
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

  if (pin_position->is_static_twice) {
    // No-op.
  } else if (pin_position->is_static) {
    // Static once. Update position.
    auto d_pos = vec_like_to_device(pin_position->curr_position, rt);
    collider->update_position(d_pos, d_pos, rt);
    pin_position->is_static_twice = true;
  } else {
    // Dynamic. Update position.
    auto d_prev = vec_like_to_device(pin_position->prev_position, rt);
    auto d_curr = vec_like_to_device(pin_position->curr_position, rt);
    collider->update_position(d_curr, d_prev, rt);
    std::swap(pin_position->curr_position, pin_position->prev_position);
    pin_position->is_static = true;
  }
}

}  // namespace silk::cuda
