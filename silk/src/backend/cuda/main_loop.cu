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
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/physical_state.cuh"
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
      assembly::assemble_cloth(registry, e, dt, state_num, rt);
      auto state = registry.get<PhysicalState>(e);
      assert(state);
      state_num += state->state_num;
      continue;
    }

    assembly::assemble_obstacle(registry, e, rt);
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
                        ctd::span<const float> curr_state,
                        ctd::span<const float> state_velocity,
                        ctd::span<float> next_state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= vert_num) {
    return;
  }

  // TODO: adaptive prediction.
  auto x = Vec3f::vec_like(curr_state.data() + 3 * tid);
  auto v = Vec3f::vec_like(state_velocity.data() + 3 * tid);

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
  auto clothes = registry.get_entity_with_components<
      PhysicalState, assembly::ClothAssemblyL1Cache, solver::ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<assembly::ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<solver::ClothADMMHelper>(e);
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
  auto clothes = registry.get_entity_with_components<
      PhysicalState, assembly::ClothAssemblyL1Cache, solver::ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<assembly::ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<solver::ClothADMMHelper>(e);
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
  if (tid == 0) {
    cu::atomic_ref<float> amin{*min_out};
    amin.fetch_min(block_min);
  }
  __syncthreads();

  float block_max = BlockReduce(tmp).Reduce(min, cu::maximum<float>{});
  if (tid == 0) {
    cu::atomic_ref<float> amax{*min_out};
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
  BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= a.size()) {
    return;
  }

  float norm2 = a[tid] - b[tid];
  norm2 = norm2 * norm2;
  float reduced = BlockReduce(tmp).Sum(norm2);
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> a_out{*norm2_out};
    a_out.fetch_add(reduced);
  }
}

__global__ void min_toi(ctd::span<const collision::Collision> collisions,
                        float* out) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= collisions.size()) {
    return;
  }

  float toi = collisions[tid].toi;
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

// void ADMMSolver::clear(ObjRegistry& registry) {
//   for (Entity& e : registry.get_all_entities()) {
//     registry.remove<ClothTopology>(e);
//     registry.remove<ClothSolverContext>(e);
//     registry.remove<ObjectState>(e);
//     registry.remove<ObjectCollider>(e);
//   }
// }
//
// void ADMMSolver::reset(ObjRegistry& registry) {
//   batch_reset_cloth_simulation(registry);
//   batch_reset_obstacle_simulation(registry);
// }

std::optional<MainLoop::Error> MainLoop::step(ObjRegistry& registry,
                                              CudaRuntime rt) {
  SPDLOG_DEBUG("solver step");

  auto curr_state = alloc<float>(rt, 0);
  auto state_velocity = alloc<float>(rt, 0);

  auto err = init(registry, curr_state, state_velocity, dt, rt);
  if (err) {
    return err;
  }

  int state_num = curr_state.size();
  auto next_state = alloc<float>(rt, state_num);
  auto tmp = alloc<float>(rt, state_num);
  float remaining_step = 1.0f;
  auto lhs_diag = alloc<float>(rt, state_num, 0);
  auto rhs = alloc<float>(rt, state_num, 0);
  auto inertia_mod = alloc<float>(rt, state_num);
  auto scalar_norm2 = alloc<float>(rt, 1);
  auto scalar_min_toi = alloc<float>(rt, 1);
  auto collision_storage =
      alloc<collision::Collision>(rt, init_narrowphase_cache_size);
  auto pin_constraints =
      solver::gather_pin_constraints(registry, state_num, rt);

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    cu::fill_bytes(rt.stream, lhs_diag, 0);
    cu::fill_bytes(rt.stream, rhs, 0);

    // Prediction based on linear velocity.
    int grid_num = div_round_up(state_num / 3, 128);
    predict<<<grid_num, 128, 0, rt.stream.get()>>>(
        state_num, dt, const_acceleration, curr_state, state_velocity,
        next_state);

    // Compute inertia mod.
    grid_num = div_round_up(state_num, 128);
    compute_inertia_mod<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, curr_state, state_velocity, const_acceleration, inertia_mod);

    cu::copy_bytes(rt.stream, curr_state, tmp);

    // if (!collisions.empty()) {
    //   enforce_barrier_constrain(barrier, d_next_state);
    //   cudaDeviceSynchronize();
    //   CHECK_CUDA(cudaGetLastError());
    // }

    // TODO: setup all constraints (collision missing)
    pin_constraints.reset_lagrange_mul(rt);

    // Inner loop: solve until the state update is small relative to scene size.
    float init_norm = 0;
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      // Sovle main.
      if (inner_it != 0) {
        cu::fill_bytes(rt.stream, lhs_diag, 0);
        cu::fill_bytes(rt.stream, rhs, 0);
        pin_constraints.eval(lhs_diag, rhs, rt);
        // TODO: dynamic rel_tol
        update_main(registry, linear_rel_tol, lhs_diag, rhs, inertia_mod,
                    next_state, rt);
      }

      update_aux_and_lagrange_mul(registry, max_lagrange_mul, next_state, rt);
      pin_constraints.update_lagrange_mul(next_state, rt);

      auto [min, max] = min_max(next_state, rt);
      // float dist = compute_L2_distance(state_num, tmp, d_next_state, tmp);
      if (!(std::isfinite(min) && std::isfinite(max))) {
        SPDLOG_ERROR("solver explodes");
        return Error::Diverge;
      }

      // Convergence check.
      grid_num = div_round_up(state_num, 128);
      scalar_write<float>(scalar_norm2.data(), 0, rt);
      diff_norm2<<<grid_num, 128, 0, rt.stream.get()>>>(next_state, curr_state,
                                                        scalar_norm2.data());
      float norm = std::sqrt(scalar_load(scalar_norm2.data(), rt));
      if (inner_it == 0) {
        init_norm = norm;
      }
      if (norm < non_linear_rel_tol * init_norm || norm < non_linear_abs_tol) {
        SPDLOG_DEBUG("||dx|| < {}, NL loop terminate", norm);
        break;
      }

      // CHECK_CUDA(cudaMemcpy(d_buffer, d_next_state, state_num *
      // sizeof(float),
      //                       cudaMemcpyDeviceToDevice));
    }

    // Project to barrier targets to prevent accumulation of small violations,
    // which could otherwise cause zero-TOI contacts in later steps.
    // if (!collisions.empty()) {
    //   enforce_barrier_constrain(barrier, d_next_state);
    //   cudaDeviceSynchronize();
    //   CHECK_CUDA(cudaGetLastError());
    // }

    // Full collision update.
    for (uint32_t e :
         registry.get_entity_with_components<PhysicalState,
                                             collision::ObjectCollider>()) {
      auto phy_state = registry.get<PhysicalState>(e);
      auto collider = registry.get<collision::ObjectCollider>(e);
      assert(phy_state && collider);

      ctd::span<const float> curr(next_state.data() + phy_state->state_offset,
                                  phy_state->state_num);
      ctd::span<const float> prev(curr_state.data() + phy_state->state_offset,
                                  phy_state->state_num);
      collider->update_position(curr, prev, rt);
    }
    auto collisions = collision::find_collision(
        registry, dt, init_broadphase_cache_size, collision_storage, rt);

    // CCD line search.
    float h_min_toi = 1.0f;
    if (!collisions.empty()) {
      SPDLOG_DEBUG("find {} collisions", collision_num);

      scalar_write(scalar_min_toi.data(), 0.0f, rt);
      int grid_num = div_round_up(collisions.size(), 128);
      min_toi<<<grid_num, 128, 0, rt.stream.get()>>>(collisions,
                                                     scalar_min_toi.data());

      // Back off to 80% of TOI as a safety margin to remain strictly
      // pre-contact and avoid zero toi.
      float h_min_toi = 0.8 * scalar_load(scalar_min_toi.data(), rt);
      SPDLOG_DEBUG("earliest toi {}", h_min_toi);
    }

    grid_num = div_round_up(state_num, 128);
    update_velocity<<<grid_num, 128, 0, rt.stream.get()>>>(
        dt, curr_state, next_state, state_velocity);

    if (h_min_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          h_min_toi, remaining_step);
      grid_num = div_round_up(state_num, 128);
      mix<<<grid_num, 128, 0, rt.stream.get()>>>(remaining_step, next_state,
                                                 curr_state, curr_state);
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", h_min_toi);
    grid_num = div_round_up(state_num, 128);
    mix<<<grid_num, 128, 0, rt.stream.get()>>>(h_min_toi, next_state,
                                               curr_state, curr_state);
    remaining_step -= h_min_toi;
    auto update_all_toi = [collisions, delta = h_min_toi] __device__(int i) {
      collisions[i].toi -= delta;
    };
    cub::DeviceFor::Bulk(collisions.size(), update_all_toi, rt.stream.get());
  }

  // Write solution back to registry
  for (auto& phy_state : registry.get_all_components<PhysicalState>()) {
    int offset = phy_state.state_offset;
    int num = phy_state.state_num;
    ctd::span<float> local_state(curr_state.data() + offset, num);
    ctd::span<float> local_vel(state_velocity.data() + offset, num);
    cu::copy_bytes(rt.stream, local_state, *phy_state.curr_state);
    cu::copy_bytes(rt.stream, local_vel, *phy_state.state_velocity);
  }

  return std::nullopt;
}

// collision::Bbox ADMMSolver::compute_scene_bbox(ObjRegistry& registry) {
//   auto& colliders = registry.get_all<ObjectCollider>();
//   assert(!colliders.empty());
//
//   Bbox bbox = colliders[0].bbox;
//   for (auto& c : colliders) {
//     bbox.merge_inplace(c.bbox);
//   }
//
//   return bbox;
// }

void ADMMSolver::compute_barrier_constrain(
    const Eigen::VectorXf& state, const std::vector<Collision>& collisions,
    BarrierConstrain& barrier) {
  assert(state.size() == barrier.state_num);

  int state_num = state.size();
  if (collisions.empty()) {
    barrier.constrain_num = 0;
    CHECK_CUDA(cudaMemset(barrier.d_lhs, 0, state_num * sizeof(float)));
    CHECK_CUDA(cudaMemset(barrier.d_rhs, 0, state_num * sizeof(float)));
    CHECK_CUDA(cudaDeviceSynchronize());
    return;
  }

  int constrain_num = 0;
  Eigen::VectorXf lhs = Eigen::VectorXf::Zero(state_num);
  Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state_num);

  for (auto& c : collisions) {
    // Zero stiffness is not expected currently; kept for future
    // non-distance-barrier update.
    if (c.stiffness == 0.0f) {
      continue;
    }

    Eigen::Vector4i offset = 3 * c.index;
    if (c.type == CollisionType::PointTriangle) {
      offset(0) += c.state_offset_a;
      offset(1) += c.state_offset_b;
      offset(2) += c.state_offset_b;
      offset(3) += c.state_offset_b;
    } else {
      offset(0) += c.state_offset_a;
      offset(1) += c.state_offset_a;
      offset(2) += c.state_offset_b;
      offset(3) += c.state_offset_b;
    }

    for (int i = 0; i < 4; ++i) {
      // If inverse mass is 0, this is either a pinned vertex or an obstacle.
      if (c.inv_mass(i) == 0.0f) {
        continue;
      }

      auto seq = Eigen::seqN(offset(i), 3);
      Eigen::Vector3f position_t0 = state(seq);
      Eigen::Vector3f reflection;

      // Compute collision reflection as target of barrier constrain.
      if (c.use_small_ms) {
        // If use_small_ms is true that means CCD detects zero toi under normal
        // minimal separation and fallbacks to a smaller one to get non-zero
        // toi. Thus we assume true toi = 0 and compute reflection aggressively.
        reflection = position_t0 + c.velocity_t1.col(i);
      } else {
        reflection = position_t0 + c.toi * c.velocity_t0.col(i) +
                     (1.0f - c.toi) * c.velocity_t1.col(i);
      }

      lhs(seq) += c.stiffness * Eigen::Vector3f::Ones();
      rhs(seq) += c.stiffness * reflection;
      // mark affected coordinate entries in lhs/rhs; indices will be
      // compacted after accumulation based on non-zero lhs entries
      constrain_num += 3;
    }
  }

  // No constraints accumulated; early exit
  // Keep parity with CPU path: if no collisions, constrain_num stays 0 earlier
  // but we already returned in that case. Here we assert on compacted size.
  assert(constrain_num != 0);
  // Build compact index list by counting non-zeros in lhs
  std::vector<int> h_indices;
  h_indices.reserve(state_num);
  for (int i = 0; i < state_num; ++i) {
    if (lhs(i) != 0.0f) {
      h_indices.push_back(i);
    }
  }
  barrier.constrain_num = static_cast<int>(h_indices.size());
  assert(barrier.constrain_num != 0);
  if (barrier.constrain_num > 0) {
    CHECK_CUDA(cudaMemcpy(barrier.d_index, h_indices.data(),
                          barrier.constrain_num * sizeof(int),
                          cudaMemcpyHostToDevice));
  }
  CHECK_CUDA(cudaMemcpy(barrier.d_lhs, lhs.data(), state_num * sizeof(float),
                        cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(barrier.d_rhs, rhs.data(), state_num * sizeof(float),
                        cudaMemcpyHostToDevice));
}

}  // namespace silk::cuda
