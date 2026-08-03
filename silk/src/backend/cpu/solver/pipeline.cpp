#include "backend/cpu/collision/pipeline.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "backend/cpu/collision/bbox.hpp"
#include "backend/cpu/collision/collision.hpp"
#include "backend/cpu/collision/object_collider.hpp"
#include "backend/cpu/ecs.hpp"
#include "backend/cpu/object_state.hpp"
#include "backend/cpu/obstacle_position.hpp"
#include "backend/cpu/solver/barrier_constrain.hpp"
#include "backend/cpu/solver/cloth_solver_context.hpp"
#include "backend/cpu/solver/cloth_solver_utils.hpp"
#include "backend/cpu/solver/obstacle_solver_utils.hpp"
#include "backend/cpu/solver/pipeline.hpp"
#include "common/logger.hpp"
#include "silk/silk.hpp"

namespace silk::cpu {

namespace {

void update_dynamic_colliders(Registry& registry,
                              const Eigen::VectorXf& current_state,
                              const Eigen::VectorXf& previous_state) {
  for (uint32_t entity : registry.get_all_entities()) {
    auto config = registry.get<CollisionConfig>(entity);
    auto state = registry.get<ObjectState>(entity);
    auto collider = registry.get<ObjectCollider>(entity);
    if (config && state && collider) {
      collider->update(*config, *state, current_state, previous_state);
    }
  }
}

}  // namespace

void SolverPipeline::reset(Registry& registry) {
  batch_reset_cloth_simulation(registry);
  batch_reset_obstacle_simulation(registry);
}

bool SolverPipeline::step(Registry& registry) {
  SPDLOG_DEBUG("solver step");

  ObjectState global_state;
  if (!init(registry, global_state)) {
    return false;
  }

  int state_num = global_state.state_num;
  if (!state_num) {
    SPDLOG_DEBUG("Nothing to solve");
    return true;
  }

  auto& curr_state = global_state.curr_state;
  auto& state_velocity = global_state.state_velocity;

  float remaining_step = 1.0f;

  // Scene bbox is used to estimate the termination criteria of the inner loop
  // and the floating-point precision of the collision pipeline.
  Bbox scene_bbox = compute_scene_bbox(curr_state);

  // Compute step invariant rhs.
  Eigen::VectorXf init_rhs = Eigen::VectorXf::Zero(state_num);
  batch_compute_cloth_invariant_rhs(registry, init_rhs);

  // Expand per-vertex acceleration (XYZ per vertex) to the packed state vector.
  Eigen::VectorXf acceleration = const_acceleration.replicate(state_num / 3, 1);

  // Init active collisions.
  update_dynamic_colliders(registry, curr_state, curr_state);
  std::vector<Collision> collisions =
      collision_pipeline.find_active_collisions(registry, 0.0f);

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    Eigen::VectorXf outer_rhs = init_rhs;
    BarrierConstrain barrier_constrain =
        compute_barrier_constrain(curr_state, collisions);
    // Prediction based on linear velocity.
    Eigen::VectorXf next_state =
        curr_state + dt * state_velocity + (dt * dt) * acceleration;

    if (!batch_compute_cloth_outer_loop(registry, curr_state, state_velocity,
                                        acceleration, barrier_constrain,
                                        outer_rhs)) {
      return false;
    }

    // Inner loop: solve until the state update is small relative to scene size.
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      Eigen::VectorXf solution(state_num);
      if (!batch_compute_cloth_inner_loop(registry, next_state, outer_rhs,
                                          solution)) {
        return false;
      }

      if (!solution.allFinite()) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }

      float scene_scale = (scene_bbox.max - scene_bbox.min).norm();
      assert(scene_scale != 0);
      float threshold = 0.05f * scene_scale;
      if ((solution - next_state).norm() <= threshold) {
        SPDLOG_DEBUG("||dx|| < {}, lg loop terminate", threshold);
        next_state = solution;
        break;
      }

      next_state = solution;
    }

    // Project to barrier targets to prevent accumulation of small violations,
    // which could otherwise cause zero-TOI contacts in later steps.
    if (!collisions.empty()) {
      enforce_barrier_constrain(barrier_constrain, scene_bbox, next_state);
    }

    // CCD rollback.
    update_dynamic_colliders(registry, next_state, curr_state);
    float time_start = 1.0f - remaining_step;
    float earliest_toi = collision_pipeline.find_earliest_toi(
        registry, scene_bbox, time_start, remaining_step);

    state_velocity = (next_state - curr_state) / dt;

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      curr_state += remaining_step * (next_state - curr_state);
      break;
    }

    // Stay strictly before the CCD separation surface.
    float rollback_toi = 0.8f * earliest_toi;
    SPDLOG_DEBUG("CCD rollback to toi {}", rollback_toi);
    curr_state = rollback_toi * (next_state - curr_state) + curr_state;
    remaining_step -= rollback_toi;

    // Refit dynamic broadphase boxes to the rolled-back state, then use DCD
    // to build the active set.
    update_dynamic_colliders(registry, curr_state, curr_state);
    float frame_time = 1.0f - remaining_step;
    collisions =
        collision_pipeline.find_active_collisions(registry, frame_time);
    SPDLOG_DEBUG("DCD found {} active collisions", collisions.size());
  }

  // Write solution back to registry
  for (auto& state : registry.get_all_components<ObjectState>()) {
    auto seq = Eigen::seqN(state.state_offset, state.state_num);
    state.curr_state = curr_state(seq);
    state.state_velocity = state_velocity(seq);
  }

  return true;
}

// Lazily init all entity and collect solver state into global array.
bool SolverPipeline::init(Registry& registry, ObjectState& global_state) {
  int state_num = 0;
  for (uint32_t e : registry.get_all_entities()) {
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      if (!prepare_cloth_simulation(registry, e, dt, state_num)) {
        return false;
      }

      auto state = registry.get<ObjectState>(e);
      assert(state);
      state_num += state->state_num;

      continue;
    }

    auto obstacle_position = registry.get<ObstaclePosition>(e);
    if (obstacle_position) {
      prepare_obstacle_simulation(registry, e);
      continue;
    }
  }

  // Gather all object state into a continuous global state array.
  global_state.state_offset = 0;
  global_state.state_num = state_num;
  global_state.curr_state.resize(state_num);
  global_state.state_velocity.resize(state_num);

  for (uint32_t e : registry.get_all_entities()) {
    auto state = registry.get<ObjectState>(e);
    if (!state) {
      continue;
    }

    // Per-entity velocity damping: scale velocities by (1 - damping).
    float damp_factor = 1.0f;
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      damp_factor = 1.0f - cloth_config->damping;
    }

    auto seq = Eigen::seqN(state->state_offset, state->state_num);
    global_state.curr_state(seq) = state->curr_state;
    global_state.state_velocity(seq) = damp_factor * state->state_velocity;
  }

  return true;
}

Bbox SolverPipeline::compute_scene_bbox(const Eigen::VectorXf& state) {
  int num = state.size();
  auto reshaped = state.reshaped(3, num / 3);
  Eigen::Vector3f min = reshaped.rowwise().minCoeff();
  Eigen::Vector3f max = reshaped.rowwise().maxCoeff();

  return Bbox{min, max};
}

BarrierConstrain SolverPipeline::compute_barrier_constrain(
    const Eigen::VectorXf& state, const std::vector<Collision>& collisions) {
  int state_num = state.size();
  Eigen::VectorXf lhs = Eigen::VectorXf::Zero(state_num);
  Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state_num);

  if (collisions.empty()) {
    return BarrierConstrain{lhs, rhs};
  }

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
      Eigen::Vector3f reflection = state(seq) + c.correction.col(i);

      lhs(seq) += c.stiffness * Eigen::Vector3f::Ones();
      rhs(seq) += c.stiffness * reflection;
    }
  }

  return BarrierConstrain{std::move(lhs), std::move(rhs)};
}

void SolverPipeline::enforce_barrier_constrain(
    const BarrierConstrain& barrier_constrain, const Bbox& scene_bbox,
    Eigen::VectorXf& state) const {
  auto& b = barrier_constrain;

  int state_num = state.size();
  float scene_scale = (scene_bbox.max - scene_bbox.min).norm();
  // Tolerance scaled by scene size to avoid sensitivity to units.
  float threshold = 1e-5f * scene_scale;

  for (int i = 0; i < state_num; ++i) {
    if (b.lhs(i) == 0.0f) {
      continue;
    }

    float target = b.rhs(i) / b.lhs(i);
    float abs_diff = std::abs(target - state(i));
    if (abs_diff > threshold) {
      SPDLOG_DEBUG("barrier constrain fails. abs diff {} > {}", abs_diff,
                   threshold);
    }

    state(i) = target;
  }
}

}  // namespace silk::cpu
