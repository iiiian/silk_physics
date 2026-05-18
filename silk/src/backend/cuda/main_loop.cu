#include "backend/cuda/main_loop.cuh"

#include <Eigen/Core>
#include <cmath>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/atomic>
#include <cuda/functional>
#include <cuda/std/limits>
#include <optional>

#include "backend/cuda/assembly/cloth_assembler.cuh"
#include "backend/cuda/assembly/obstacle_assembler.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/find_collision.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/barrier_constraints.cuh"
#include "backend/cuda/solver/cloth_admm_helper.cuh"
#include "backend/cuda/solver/equality_constraints.cuh"
#include "backend/cuda/solver/pin_constraints.cuh"
#include "common/logger.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

namespace {

/// @brief Lazily init all entity and collect solver state into global array.
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

  // TODO: adaptive prediction.
  auto x = Vec3f::vec_like(state.data() + 3 * tid);
  auto v = Vec3f::vec_like(velocity.data() + 3 * tid);

  // x_next = x + dt*velocity + dt*dt*acceleration.
  Vec3f next;
  next = axpby(1.0, x, dt, v);
  next = axpby(1.0, next, dt * dt, acc);

#pragma unroll
  for (int i = 0; i < 3; ++i) {
    next_state[3 * tid + i] = next(i);
  }
}

void update_aux_and_lagrange_mul(ObjRegistry& registry, float max_lagrange_mul,
                                 ctd::span<const float> state, CudaRuntime rt) {
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<ClothADMMHelper>(e);
    assert(phy_state && l1_cache && admm_helper);

    auto x = state.subspan(phy_state->state_offset, phy_state->state_num);
    admm_helper->update_aux_var_and_lagrange_mul(max_lagrange_mul, *l1_cache, x,
                                                 rt);
  }
}

void update_main(ObjRegistry& registry, float rel_tol,
                 ctd::span<const float> lhs_diag, ctd::span<const float> rhs,
                 ctd::span<const float> inertia_mod, ctd::span<float> state,
                 CudaRuntime rt) {
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<ClothADMMHelper>(e);
    assert(phy_state && l1_cache && admm_helper);

    auto x = state.subspan(phy_state->state_offset, phy_state->state_num);
    admm_helper->solve_main_var(rel_tol, *l1_cache, lhs_diag, rhs, inertia_mod,
                                x, rt);
  }
}

__global__ void min_max_kernel(ctd::span<const float> values, float* min_out,
                               float* max_out) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float min = ctd::numeric_limits<float>::infinity();
  float max = -ctd::numeric_limits<float>::infinity();
  if (tid < values.size()) {
    min = values[tid];
    max = values[tid];
  }

  float block_min = BlockReduce(tmp).Reduce(min, cu::minimum<float>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> amin{*min_out};
    amin.fetch_min(block_min);
  }
  __syncthreads();

  float block_max = BlockReduce(tmp).Reduce(max, cu::maximum<float>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> amax{*max_out};
    amax.fetch_max(block_max);
  }
}

std::pair<float, float> min_max(ctd::span<const float> values, CudaRuntime rt) {
  auto d_min = alloc<float>(rt, 1, ctd::numeric_limits<float>::infinity());
  auto d_max = alloc<float>(rt, 1, -ctd::numeric_limits<float>::infinity());

  int grid_num = div_round_up(values.size(), 128);
  min_max_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(values, d_min.data(),
                                                        d_max.data());
  // scalar load already sync stream.
  float h_min = scalar_load(d_min.data(), rt);
  float h_max = scalar_load(d_max.data(), rt);
  return std::make_pair(h_min, h_max);
}

__global__ void compute_inertia_mod(float dt, ctd::span<const float> state,
                                    ctd::span<const float> velocity, Vec3f acc,
                                    ctd::span<float> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= state.size()) {
    return;
  }

  float a = acc(tid % 3);
  out[tid] = -state[tid] / (dt * dt) - velocity[tid] / dt + a;
}

__global__ void diff_norm2(ctd::span<const float> a, ctd::span<const float> b,
                           float* norm2_out) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float norm2 = 0.0f;
  if (tid < a.size()) {
    float diff = a[tid] - b[tid];
    norm2 = diff * diff;
  }

  float reduced = BlockReduce(tmp).Sum(norm2);
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> a_out{*norm2_out};
    a_out.fetch_add(reduced);
  }
}

__global__ void min_toi(ctd::span<const Collision> collisions, float* out) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float toi = 1.0f;
  if (tid < collisions.size()) {
    toi = collisions[tid].toi;
  }

  float reduced = BlockReduce(tmp).Reduce(toi, cu::minimum<>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> a_out{*out};
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
  if (err) {
    return err;
  }

  // prev_state/velocity: state at previous time step.
  // outer_state/velocity: current state in outer loop.
  // inner_state: current state in inner loop.
  // inner_tmp: intermediate state in inner loop.

  int state_num = prev_state.size();
  auto outer_state = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_state, outer_state);
  auto outer_velocity = alloc<float>(rt, state_num);
  cu::copy_bytes(rt.stream, prev_velocity, outer_velocity);
  auto inner_state = alloc<float>(rt, state_num);
  auto inner_tmp = alloc<float>(rt, state_num);

  float remaining_step = 1.0f;
  auto lhs_diag = alloc<float>(rt, state_num, 0);
  auto rhs = alloc<float>(rt, state_num, 0);
  auto inertia_mod = alloc<float>(rt, state_num);
  auto scalar_norm2 = alloc<float>(rt, 1);
  auto scalar_min_toi = alloc<float>(rt, 1);
  auto collision_storage = alloc<Collision>(rt, init_narrowphase_cache_size);
  auto pin_constraints = gather_pin_constraints(registry, state_num, rt);
  std::optional<EqualityConstraints> barrier_constraints;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    // Prediction based on linear velocity.
    int vert_num = state_num / 3;
    int grid_num = div_round_up(state_num / 3, 128);
    predict<<<grid_num, 128, 0, rt.stream.get()>>>(
        vert_num, dt, const_acceleration, outer_state, outer_velocity,
        inner_state);

    // Compute inertia mod.
    grid_num = div_round_up(state_num, 128);
    compute_inertia_mod<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, prev_state, prev_velocity, const_acceleration, inertia_mod);

    pin_constraints.reset_lagrange_mul(rt);
    if (barrier_constraints) {
      barrier_constraints->reset_lagrange_mul(rt);
    }

    // Inner loop.
    float init_norm = 0;
    cu::copy_bytes(rt.stream, inner_state, inner_tmp);
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      // Sovle main.
      if (inner_it != 0) {
        cu::fill_bytes(rt.stream, lhs_diag, 0);
        cu::fill_bytes(rt.stream, rhs, 0);
        pin_constraints.eval(lhs_diag, rhs, rt);
        if (barrier_constraints) {
          barrier_constraints->eval(lhs_diag, rhs, rt);
        }
        // TODO: dynamic rel_tol
        update_main(registry, linear_rel_tol, lhs_diag, rhs, inertia_mod,
                    inner_tmp, rt);
      }

      update_aux_and_lagrange_mul(registry, max_lagrange_mul, inner_tmp, rt);
      pin_constraints.update_lagrange_mul(inner_tmp, rt);
      if (barrier_constraints) {
        barrier_constraints->update_lagrange_mul(inner_tmp, rt);
      }

      auto [min, max] = min_max(inner_tmp, rt);
      if (!(std::isfinite(min) && std::isfinite(max))) {
        SPDLOG_ERROR("solver explodes");
        return Error::Diverge;
      }

      // Convergence check.
      grid_num = div_round_up(state_num, 128);
      scalar_write<float>(scalar_norm2.data(), 0, rt);
      diff_norm2<<<grid_num, 128, 0, rt.stream.get()>>>(inner_tmp, inner_state,
                                                        scalar_norm2.data());
      float norm = std::sqrt(scalar_load(scalar_norm2.data(), rt));
      if (inner_it == 0) {
        init_norm = norm;
      }
      if (norm < non_linear_rel_tol * init_norm || norm < non_linear_abs_tol) {
        SPDLOG_DEBUG("||dx|| < {}, NL loop terminate", norm);
        cu::copy_bytes(rt.stream, inner_tmp, inner_state);
        break;
      }

      cu::copy_bytes(rt.stream, inner_tmp, inner_state);
    }

    // Project to barrier targets to prevent accumulation of small violations,
    // which could otherwise cause zero-TOI contacts in later steps.
    // TODO: maybe not required anymore?
    pin_constraints.enforce(inner_state, rt);
    if (barrier_constraints) {
      barrier_constraints->enforce(inner_state, rt);
    }

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
    if (!collisions.empty()) {
      SPDLOG_DEBUG("find {} collisions", collisions.size());

      scalar_write(scalar_min_toi.data(), 1.0f, rt);
      int grid_num = div_round_up(collisions.size(), 128);
      min_toi<<<grid_num, 128, 0, rt.stream.get()>>>(collisions,
                                                     scalar_min_toi.data());

      // Back off to 80% of TOI as a safety margin to remain strictly
      // pre-contact and avoid zero toi.
      h_min_toi = 0.8f * scalar_load(scalar_min_toi.data(), rt);
      SPDLOG_DEBUG("earliest toi {}", h_min_toi);
    }

    grid_num = div_round_up(state_num, 128);
    update_velocity<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, outer_state, inner_state, outer_velocity);

    if (h_min_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          h_min_toi, remaining_step);
      grid_num = div_round_up(state_num, 128);
      mix<<<grid_num, 128, 0, rt.stream.get()>>>(remaining_step, inner_state,
                                                 outer_state, outer_state);
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", h_min_toi);
    grid_num = div_round_up(state_num, 128);
    mix<<<grid_num, 128, 0, rt.stream.get()>>>(h_min_toi, inner_state,
                                               outer_state, outer_state);
    remaining_step -= h_min_toi;
    auto update_all_toi = [collisions, delta = h_min_toi] __device__(int i) {
      collisions[i].toi -= delta;
    };
    cub::DeviceFor::Bulk(collisions.size(), update_all_toi, rt.stream.get());

    barrier_constraints = gather_barrier_constraints(state_num, collisions, rt);
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

  return std::nullopt;
}

}  // namespace silk::cuda
