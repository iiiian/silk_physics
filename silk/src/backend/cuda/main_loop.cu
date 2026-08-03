#include "backend/cuda/main_loop.cuh"

#include <cub/cub.cuh>
#include <memory>
#include <optional>

#include "backend/cuda/assembly/cloth_assembler.cuh"
#include "backend/cuda/assembly/obstacle_assembler.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/find_collision.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/barrier_constraints.cuh"
#include "backend/cuda/solver/pin_constraints.cuh"
#include "common/logger.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

namespace {

std::optional<MainLoop::Error> init(ObjRegistry& registry,
                                    cu::device_buffer<float>& global_state,
                                    cu::device_buffer<float>& global_velocity,
                                    float dt, CudaRuntime rt) {
  int state_num = 0;
  for (uint32_t e : registry.get_all_entities()) {
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      assemble_cloth(registry, e, dt, state_num, rt);
      auto state = registry.get<PhysicalState>(e);
      assert(state);
      state_num += state->state_num;
      continue;
    }

    assemble_obstacle(registry, e, rt);
  }

  if (state_num == 0) {
    return MainLoop::Error::NothingToSolve;
  }

  // Gather all object state into a continuous global state array.
  global_state = alloc<float>(rt, state_num);
  global_velocity = alloc<float>(rt, state_num);

  for (uint32_t e : registry.get_entity_with_components<PhysicalState>()) {
    auto state = registry.get<PhysicalState>(e);
    assert(state);

    int state_num = state->state_num;
    int offset = state->state_offset;
    ctd::span<float> target_state(global_state.data() + offset, state_num);
    cu::copy_bytes(rt.stream, *state->curr_state, target_state);

    // Per-entity velocity damping: scale velocities by (1 - damping).
    float damp_factor = 1.0f;
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      damp_factor = 1.0f - cloth_config->damping;
    }

    ctd::span<float> target_vel(global_velocity.data() + offset, state_num);
    ctd::span<float> src_vel = *state->state_velocity;
    auto damp_velocity = [target_vel, src_vel, damp_factor] __device__(int i) {
      target_vel[i] = damp_factor * src_vel[i];
    };

    cub::DeviceFor::Bulk(target_vel.size(), damp_velocity, rt.stream.get());
  }

  return std::nullopt;
}

__global__ void predict(int vert_num, float dt, Vec3f acc,
                        ctd::span<const float> state,
                        ctd::span<const float> velocity,
                        ctd::span<float> next_state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= vert_num) {
    return;
  }

  auto x = Vec3f::vec_like(state.data() + 3 * tid);
  auto v = Vec3f::vec_like(velocity.data() + 3 * tid);

  Vec3f next;
  next = axpby(1.0, x, dt, v);
  next = axpby(1.0, next, dt * dt, acc);

#pragma unroll
  for (int i = 0; i < 3; ++i) {
    next_state[3 * tid + i] = next(i);
  }
}

__global__ void update_velocity(float dt, ctd::span<const float> curr_state,
                                ctd::span<const float> next_state,
                                ctd::span<float> velocity_out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= curr_state.size()) {
    return;
  }

  velocity_out[tid] = (next_state[tid] - curr_state[tid]) / dt;
}

// out = w*a + (1-w)*b
__global__ void mix(float w, ctd::span<const float> a, ctd::span<const float> b,
                    ctd::span<float> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= a.size()) {
    return;
  }

  out[tid] = w * a[tid] + (1.0 - w) * b[tid];
}

void update_collision_state(ObjRegistry& registry, ctd::span<const float> curr,
                            ctd::span<const float> prev, CudaRuntime rt) {
  for (uint32_t e :
       registry.get_entity_with_components<PhysicalState, ObjectCollider>()) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto collider = registry.get<ObjectCollider>(e);
    assert(phy_state && collider);

    ctd::span<const float> local_curr(curr.data() + phy_state->state_offset,
                                      phy_state->state_num);
    ctd::span<const float> local_prev(prev.data() + phy_state->state_offset,
                                      phy_state->state_num);
    collider->update_position(local_curr, local_prev, rt);
  }
}

}  // namespace

std::optional<MainLoop::Error> MainLoop::step(ObjRegistry& registry,
                                              CudaRuntime rt) {
  SPDLOG_DEBUG("solver step");

  auto prev_state = alloc<float>(rt, 0);
  auto prev_velocity = alloc<float>(rt, 0);
  auto err = init(registry, prev_state, prev_velocity, dt, rt);
  rt.stream.sync();
  if (err) {
    return err;
  }

  auto object_colliders = registry.get_all_components<ObjectCollider>();
  assert(!object_colliders.empty());
  float scene_bbox_padding = object_colliders[0].bbox_padding;
  for (const ObjectCollider& collider : object_colliders) {
    scene_bbox_padding = ctd::min(scene_bbox_padding, collider.bbox_padding);
  }
  float ccd_minimum_separation =
      ccd_minimum_separation_scale * scene_bbox_padding;
  float dcd_activation_distance =
      dcd_activation_distance_scale * scene_bbox_padding;

  int state_num = prev_state.size();
  auto outer_state = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_state, outer_state);
  auto outer_velocity = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_velocity, outer_velocity);
  auto inner_state = alloc<float>(rt, state_num);
  auto collision_storage = alloc<Collision>(rt, init_narrowphase_cache_size);
  auto pin_constraints = gather_pin_constraints(registry, state_num, rt);

  // DCD owns the active set. Initialize it at the beginning of the frame.
  update_collision_state(registry, outer_state, outer_state, rt);
  auto collisions =
      find_active_collisions(registry, init_broadphase_cache_size, 0.0f,
                             dcd_activation_distance, collision_storage, rt);
  std::unique_ptr<EqualityConstraints> barrier_constraints;
  if (!collisions.empty()) {
    barrier_constraints = std::make_unique<EqualityConstraints>(
        gather_barrier_constraints(state_num, collisions, rt));
  }

  float remaining_step = 1.0f;
  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    int vert_num = state_num / 3;
    int grid_num = div_round_up(vert_num, 128);
    predict<<<grid_num, 128, 0, rt.stream.get()>>>(
        vert_num, dt, const_acceleration, outer_state, outer_velocity,
        inner_state);

    auto admm_err = admm_solver.solve(
        registry, prev_state, prev_velocity, inner_state, pin_constraints,
        barrier_constraints.get(), dt, const_acceleration, rt);
    if (admm_err) {
      return Error::Diverge;
    }
    // if (barrier_constraints) {
    //   barrier_constraints->enforce(inner_state, rt);
    // }
    rt.stream.sync();

    // CCD only determines the safe line-search interval.
    update_collision_state(registry, inner_state, outer_state, rt);
    float time_start = 1.0f - remaining_step;
    float min_toi = find_min_toi(
        registry, init_broadphase_cache_size, time_start, remaining_step,
        ccd_minimum_separation, ccd_tolerance, ccd_max_iter, rt);

    grid_num = div_round_up(state_num, 128);
    update_velocity<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, outer_state, inner_state, outer_velocity);

    if (min_toi >= remaining_step) {
      SPDLOG_INFO(
          "Outer it {}. Earliest toi {} >= remaining step {}. terminate.",
          outer_it, min_toi, remaining_step);
      mix<<<grid_num, 128, 0, rt.stream.get()>>>(remaining_step, inner_state,
                                                 outer_state, outer_state);
      break;
    }

    float rollback_toi = 0.8f * min_toi;
    SPDLOG_INFO("Outer it {}, CCD rollback to toi {}", outer_it, rollback_toi);
    mix<<<grid_num, 128, 0, rt.stream.get()>>>(rollback_toi, inner_state,
                                               outer_state, outer_state);
    remaining_step -= rollback_toi;

    // Refit to the rolled-back state, then rebuild active contacts with DCD.
    update_collision_state(registry, outer_state, outer_state, rt);
    float time = 1.0f - remaining_step;
    collisions =
        find_active_collisions(registry, init_broadphase_cache_size, time,
                               dcd_activation_distance, collision_storage, rt);
    if (collisions.empty()) {
      barrier_constraints.reset();
    } else {
      barrier_constraints = std::make_unique<EqualityConstraints>(
          gather_barrier_constraints(state_num, collisions, rt));
    }
  }

  // Write solution back to registry.
  for (auto& physical_state : registry.get_all_components<PhysicalState>()) {
    int offset = physical_state.state_offset;
    int state_num = physical_state.state_num;
    ctd::span<float> local_state(outer_state.data() + offset, state_num);
    ctd::span<float> local_velocity(outer_velocity.data() + offset, state_num);
    cu::copy_bytes(rt.stream, local_state, *physical_state.curr_state);
    cu::copy_bytes(rt.stream, local_velocity, *physical_state.state_velocity);
  }

  // Update pin position static status.
  for (auto& pin : registry.get_all_components<PinPosition>()) {
    pin.prev_position = pin.curr_position;
    if (pin.is_static) {
      pin.is_static_twice = true;
    } else {
      pin.is_static = true;
    }
  }

  rt.stream.sync();
  return std::nullopt;
}

}  // namespace silk::cuda
