#include "backend/cuda/assembly/obstacle_assembler.cuh"

#include <cassert>
#include <cuda/buffer>
#include <span>
#include <utility>

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
    pin_pos = registry.set(e, PinPosition{*pin_index, pos_span});
  }
  assert(pin_pos != nullptr);

  // Ensure collider exists and is up-to-date
  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    ctd::span<const float> init_pos(init_state->position.data(),
                                    init_state->position.size());
    auto new_collider =
        ObjectCollider::from_obstacle(*config, *mesh, init_pos, rt);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  }
  assert(collider != nullptr);

  if (config->is_updated) {
    collider->update_collision_config(*config, rt);
    config->is_updated = false;
  }

  if (pin_pos->is_static_twice) {
    // No-op.
  } else if (pin_pos->is_static) {
    // Static once. Update position.
    auto d_pos = vec_like_to_device<float>(pin_pos->curr_position, rt);
    collider->update_position(d_pos, d_pos, rt);
  } else {
    // Dynamic. Update position.
    auto d_prev = vec_like_to_device<float>(pin_pos->prev_position, rt);
    auto d_curr = vec_like_to_device<float>(pin_pos->curr_position, rt);
    collider->update_position(d_curr, d_prev, rt);
  }
}

}  // namespace silk::cuda
