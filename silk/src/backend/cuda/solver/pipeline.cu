#include "backend/cuda/solver/pipeline.cuh"

#include <Eigen/Core>
#include <cmath>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <optional>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/obstacle_position.hpp"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/cloth_solver_utils.cuh"
#include "backend/cuda/solver/obstacle_solver_utils.cuh"
#include "common/logger.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

namespace {

/// @brief Lazily init all entity and collect solver state into global array.
std::optional<ADMMError> init(ObjRegistry& registry,
                              cu::device_buffer<float>& global_state,
                              cu::device_buffer<float>& global_velocity,
                              float dt, CudaRuntime rt) {
  int state_num = 0;
  for (uint32_t e : registry.get_all_entities()) {
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      prepare_cloth_simulation(registry, e, dt, state_num, rt);
      auto state = registry.get<PhysicalState>(e);
      assert(state);
      state_num += state->state_num;
      continue;
    }

    auto obstacle_position = registry.get<ObstaclePosition>(e);
    if (obstacle_position) {
      prepare_obstacle_simulation(registry, e, rt);
      continue;
    }
  }

  if (state_num == 0) {
    return ADMMError::NothingToSolve;
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

std::optional<ADMMError> ADMMSolver::step(ObjRegistry& registry,
                                          CudaRuntime rt) {
  SPDLOG_DEBUG("solver step");

  auto global_state = alloc<float>(rt, 0);
  auto global_vel = alloc<float>(rt, 0);

  auto err = init(registry, global_state, global_vel, dt, rt);
  if (err) {
    return err;
  }

  int state_num = global_state.size();
  auto next_state = alloc<float>(rt, state_num);
  auto buffer = alloc<float>(rt, state_num);
  // std::vector<Collision> collisions;
  // BarrierConstrain barrier{state_num};
  float remaining_step = 1.0f;
  // Bbox scene_bbox = compute_scene_bbox(registry);
  auto init_rhs = alloc<float>(rt, state_num);
  // batch_compute_cloth_invariant_rhs(registry, init_rhs);
  auto outer_rhs = alloc<float>(rt, state_num);

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    CHECK_CUDA(cudaMemcpy(outer_rhs, init_rhs, state_num * sizeof(float),
                          cudaMemcpyDeviceToDevice));
    compute_barrier_constrain(curr_state, collisions, barrier);

    // Prediction based on linear velocity.
    predict(state_num, dt, const_acceleration(0), const_acceleration(1),
            const_acceleration(2), d_curr_state, d_state_velocity,
            d_next_state);
    CHECK_CUDA(cudaMemcpy(d_buffer, d_next_state, state_num * sizeof(float),
                          cudaMemcpyDeviceToDevice));

    if (!batch_compute_cloth_outer_loop(registry, d_curr_state,
                                        d_state_velocity, barrier,
                                        const_acceleration, outer_rhs)) {
      return false;
    }

    if (!collisions.empty()) {
      enforce_barrier_constrain(barrier, d_next_state);
      cudaDeviceSynchronize();
      CHECK_CUDA(cudaGetLastError());
    }

    // Inner loop: solve until the state update is small relative to scene size.
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      if (!batch_compute_cloth_inner_loop(registry, outer_rhs, barrier,
                                          d_next_state)) {
        return false;
      }

      float threshold = 0.05f * (scene_bbox.max - scene_bbox.min).norm();
      assert(threshold != 0);
      float dist =
          compute_L2_distance(state_num, d_buffer, d_next_state, d_buffer);
      if (!std::isfinite(dist)) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }
      if (dist <= threshold) {
        SPDLOG_DEBUG("||dx|| < {}, lg loop terminate", threshold);
        break;
      }

      CHECK_CUDA(cudaMemcpy(d_buffer, d_next_state, state_num * sizeof(float),
                            cudaMemcpyDeviceToDevice));
    }

    // Project to barrier targets to prevent accumulation of small violations,
    // which could otherwise cause zero-TOI contacts in later steps.
    if (!collisions.empty()) {
      enforce_barrier_constrain(barrier, d_next_state);
      cudaDeviceSynchronize();
      CHECK_CUDA(cudaGetLastError());
    }

    // Full collision update.
    CHECK_CUDA(cudaMemcpy(curr_state.data(), d_curr_state,
                          state_num * sizeof(float), cudaMemcpyDeviceToHost));
    CHECK_CUDA(cudaMemcpy(h_next_state.data(), d_next_state,
                          state_num * sizeof(float), cudaMemcpyDeviceToHost));
    for (Entity& e : registry.get_all_entities()) {
      auto config = registry.get<CollisionConfig>(e);
      auto state = registry.get<ObjectState>(e);
      auto collider = registry.get<ObjectCollider>(e);
      if (!(config && state && collider)) {
        continue;
      }
      collider->update(*config, *state, h_next_state, h_curr_state);
    }
    collisions = collision_pipeline.find_collision(registry, scene_bbox, dt);

    // CCD line search over the remaining normalized substep.
    float earliest_toi = 1.0f;
    if (!collisions.empty()) {
      SPDLOG_DEBUG("find {} collisions", collisions.size());
      for (auto& c : collisions) {
        earliest_toi = std::min(earliest_toi, c.toi);
      }
      // Back off to 80% of TOI as a safety margin to remain strictly
      // pre-contact and avoid zero toi.
      earliest_toi *= 0.8f;
      SPDLOG_DEBUG("earliest toi {}", earliest_toi);
    }

    update_velocity(state_num, dt, d_curr_state, d_next_state,
                    d_state_velocity);
    cudaDeviceSynchronize();
    CHECK_CUDA(cudaGetLastError());

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      vec_mix(state_num, 1.0f - remaining_step, d_curr_state, d_next_state,
              d_curr_state);
      curr_state =
          (1.0f - remaining_step) * curr_state + remaining_step * h_next_state;
      cudaDeviceSynchronize();
      CHECK_CUDA(cudaGetLastError());
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", earliest_toi);
    vec_mix(state_num, 1.0f - earliest_toi, d_curr_state, d_next_state,
            d_curr_state);
    curr_state =
        (1.0f - earliest_toi) * curr_state + earliest_toi * h_next_state;
    remaining_step -= earliest_toi;
    for (auto& c : collisions) {
      c.toi -= earliest_toi;
    }
    cudaDeviceSynchronize();
    CHECK_CUDA(cudaGetLastError());
  }

  // Write solution back to registry
  for (auto& state : registry.get_all<ObjectState>()) {
    int offset = state.state_offset;
    int num = state.state_num;
    CHECK_CUDA(cudaMemcpy(state.d_curr_state, d_curr_state + offset,
                          num * sizeof(float), cudaMemcpyDeviceToDevice));
    CHECK_CUDA(cudaMemcpy(state.d_state_velocity, d_state_velocity + offset,
                          num * sizeof(float), cudaMemcpyDeviceToDevice));
  }

  return true;
}

Bbox ADMMSolver::compute_scene_bbox(ObjRegistry& registry) {
  auto& colliders = registry.get_all<ObjectCollider>();
  assert(!colliders.empty());

  Bbox bbox = colliders[0].bbox;
  for (auto& c : colliders) {
    bbox.merge_inplace(c.bbox);
  }

  return bbox;
}

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
