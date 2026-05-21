#include "backend/cuda/main_loop.cuh"

#include <cub/cub.cuh>
#include <cuda/atomic>
#include <cuda/functional>
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

__global__ void collision_stats(ctd::span<const Collision> collisions,
                                float* min_toi_out, int* line_collision_count,
                                int* initial_contact_count) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float toi = 1.0f;
  if (tid < collisions.size()) {
    if (collisions[tid].is_initial_contact) {
      atomicAdd(initial_contact_count, 1);
    } else {
      atomicAdd(line_collision_count, 1);
      toi = collisions[tid].toi;
    }
  }

  float reduced = BlockReduce(tmp).Reduce(toi, cu::minimum<>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> a_out{*min_toi_out};
    a_out.fetch_min(reduced);
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

  int state_num = prev_state.size();
  auto outer_state = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_state, outer_state);
  auto outer_velocity = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_velocity, outer_velocity);
  auto inner_state = alloc<float>(rt, state_num);

  float remaining_step = 1.0f;
  auto scalar_min_toi = alloc<float>(rt, 1);
  auto scalar_line_collision_count = alloc<int>(rt, 1);
  auto scalar_initial_contact_count = alloc<int>(rt, 1);
  auto collision_storage = alloc<Collision>(rt, init_narrowphase_cache_size);
  auto pin_constraints = gather_pin_constraints(registry, state_num, rt);
  std::unique_ptr<EqualityConstraints> barrier_constraints;
  bool solved_initial_contacts = false;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_INFO("Outer iter {}", outer_it);

    // Prediction based on linear velocity.
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

    rt.stream.sync();

    // Full collision update.
    for (uint32_t e :
         registry.get_entity_with_components<PhysicalState, ObjectCollider>()) {
      auto phy_state = registry.get<PhysicalState>(e);
      auto collider = registry.get<ObjectCollider>(e);
      assert(phy_state && collider);

      ctd::span<const float> curr(inner_state.data() + phy_state->state_offset,
                                  phy_state->state_num);
      ctd::span<const float> prev(outer_state.data() + phy_state->state_offset,
                                  phy_state->state_num);
      collider->update_position(curr, prev, rt);
    }
    auto collisions = find_collision(registry, dt, init_broadphase_cache_size,
                                     collision_storage, rt);

    // CCD line search.
    float h_min_toi = 1.0f;
    int h_line_collision_count = 0;
    int h_initial_contact_count = 0;
    if (!collisions.empty()) {
      SPDLOG_DEBUG("find {} collisions", collisions.size());

      scalar_write(scalar_min_toi.data(), 1.0f, rt);
      scalar_write(scalar_line_collision_count.data(), 0, rt);
      scalar_write(scalar_initial_contact_count.data(), 0, rt);
      int grid_num = div_round_up(collisions.size(), 128);
      collision_stats<<<grid_num, 128, 0, rt.stream.get()>>>(
          collisions, scalar_min_toi.data(), scalar_line_collision_count.data(),
          scalar_initial_contact_count.data());

      h_line_collision_count =
          scalar_load(scalar_line_collision_count.data(), rt);
      h_initial_contact_count =
          scalar_load(scalar_initial_contact_count.data(), rt);
      if (h_line_collision_count > 0) {
        // Back off to 80% of TOI as a safety margin to remain strictly
        // pre-contact and avoid zero toi.
        h_min_toi = 0.8f * scalar_load(scalar_min_toi.data(), rt);
      }
      SPDLOG_INFO("collision stats line={} initial={} earliest toi {}",
                  h_line_collision_count, h_initial_contact_count, h_min_toi);
      if (h_min_toi == 0.0) {
        throw std::runtime_error("Zero toi");
      }
    }

    grid_num = div_round_up(state_num, 128);
    update_velocity<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, outer_state, inner_state, outer_velocity);

    if (h_initial_contact_count > 0 && !solved_initial_contacts) {
      SPDLOG_INFO("solve {} initial contacts at current outer state",
                  h_initial_contact_count);
      barrier_constraints =
          std::make_unique<EqualityConstraints>(gather_barrier_constraints(
              state_num, collisions, rt,
              BarrierCollisionFilter::InitialContactsOnly));
      solved_initial_contacts = true;
      continue;
    }

    if (h_min_toi >= remaining_step) {
      SPDLOG_INFO(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          h_min_toi, remaining_step);
      grid_num = div_round_up(state_num, 128);
      mix<<<grid_num, 128, 0, rt.stream.get()>>>(remaining_step, inner_state,
                                                 outer_state, outer_state);
      break;
    }

    SPDLOG_INFO("CCD rollback to toi {}", h_min_toi);
    grid_num = div_round_up(state_num, 128);
    mix<<<grid_num, 128, 0, rt.stream.get()>>>(h_min_toi, inner_state,
                                               outer_state, outer_state);
    remaining_step -= h_min_toi;
    auto update_all_toi = [collisions, delta = h_min_toi] __device__(int i) {
      if (!collisions[i].is_initial_contact) {
        collisions[i].toi -= delta;
      }
    };
    cub::DeviceFor::Bulk(collisions.size(), update_all_toi, rt.stream.get());

    auto barrier_filter = BarrierCollisionFilter::All;
    if (h_initial_contact_count > 0 && h_line_collision_count > 0) {
      barrier_filter = BarrierCollisionFilter::LineSearchOnly;
    }
    barrier_constraints = std::make_unique<EqualityConstraints>(
        gather_barrier_constraints(state_num, collisions, rt, barrier_filter));
    solved_initial_contacts = false;
  }

  // Write solution back to registry
  for (auto& phy_state : registry.get_all_components<PhysicalState>()) {
    int offset = phy_state.state_offset;
    int num = phy_state.state_num;
    ctd::span<float> local_state(outer_state.data() + offset, num);
    ctd::span<float> local_vel(outer_velocity.data() + offset, num);
    cu::copy_bytes(rt.stream, local_state, *phy_state.curr_state);
    cu::copy_bytes(rt.stream, local_vel, *phy_state.state_velocity);
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
