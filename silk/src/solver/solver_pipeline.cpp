#include "solver_pipeline.hpp"

#include <Eigen/Core>
#include <vector>

#include "../collision.hpp"
#include "../collision_pipeline.hpp"
#include "../ecs.hpp"
#include "../logger.hpp"
#include "../object_collider_utils.hpp"
#include "cloth_solver.hpp"

namespace silk {

void SolverPipeline::clear(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    registry.remove<ClothStaticSolverData>(e);
    registry.remove<ClothDynamicSolverData>(e);
    registry.remove<SolverState>(e);
    registry.remove<ObjectCollider>(e);
  }

  collisions_.clear();
}

void SolverPipeline::reset(Registry& registry) {
  reset_all_cloth_for_solver(registry);
  collisions_.clear();
}

bool SolverPipeline::step(Registry& registry,
                          CollisionPipeline& collision_pipeline) {
  SPDLOG_DEBUG("solver step");

  if (!init_all_cloth_for_solver(registry, dt)) {
    return false;
  }

  // Collect state and state velocity of all physical entities into one global
  // state and velocity.
  auto [state_offset, state_num, curr_state, state_velocity] =
      compute_global_state(registry);
  if (state_num == 0) {
    SPDLOG_DEBUG("nothing to solve");
    return true;
  }

  // Build/update colliders for objects and obstacles before solving.
  make_all_object_collider(registry);
  update_all_obstacle_object_collider(registry);

  // Scene bbox is used to estimate the termination criteria of the inner loop
  // and the floating-point precision of the collision pipeline.
  Bbox scene_bbox = compute_scene_bbox(curr_state);

  // Clean up collisions involving removed entities.
  cleanup_collisions(registry);

  // compute init rhs for all physical entities
  Eigen::VectorXf init_rhs;
  init_rhs.resize(state_num);
  compute_all_cloth_init_rhs(registry, init_rhs);

  // Expand per-vertex acceleration (XYZ per vertex) to the packed state vector.
  Eigen::VectorXf acceleration = const_acceleration.replicate(state_num / 3, 1);
  float remaining_step = 1.0f;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    Eigen::VectorXf outer_rhs = init_rhs;
    BarrierConstrain barrier_constrain = compute_barrier_constrain(curr_state);
    // Prediction based on linear velocity.
    Eigen::VectorXf next_state =
        curr_state + dt * state_velocity + (dt * dt) * acceleration;

    if (!compute_all_cloth_outer_loop(registry, curr_state, state_velocity,
                                      acceleration, barrier_constrain,
                                      outer_rhs)) {
      return false;
    }

    // Inner loop: solve until the state update is small relative to scene size.
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      Eigen::VectorXf solution(state_num);
      if (!compute_all_cloth_inner_loop(registry, next_state, outer_rhs,
                                        solution)) {
        return false;
      }

      if (!solution.allFinite()) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }

      // Termination threshold relative to scene extent (5% of diagonal).
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
    enforce_barrier_constrain(barrier_constrain, scene_bbox, next_state);

    // Full collision update.
    update_all_physical_object_collider(registry, next_state, curr_state);
    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), scene_bbox, dt);

    // CCD line search over the remaining normalized substep.
    float earliest_toi = 1.0f;
    if (!collisions_.empty()) {
      SPDLOG_DEBUG("find {} collisions", collisions_.size());
      for (auto& c : collisions_) {
        earliest_toi = std::min(earliest_toi, c.toi);
      }
      // Back off to 80% of TOI as a safety margin to remain strictly
      // pre-contact and avoid zero toi.
      earliest_toi *= 0.8f;
      SPDLOG_DEBUG("earliest toi {}", earliest_toi);
    }

    state_velocity = (next_state - curr_state) / dt;

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      curr_state += remaining_step * (next_state - curr_state);
      for (auto& c : collisions_) {
        c.toi -= remaining_step;
      }
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", earliest_toi);
    curr_state = earliest_toi * (next_state - curr_state) + curr_state;
    remaining_step -= earliest_toi;
    for (auto& c : collisions_) {
      c.toi -= earliest_toi;
    }
  }

  // Write solution back to registry.
  for (auto& state : registry.get_all<SolverState>()) {
    auto seq = Eigen::seqN(state.state_offset, state.state_num);
    state.curr_state = curr_state(seq);
    state.state_velocity = state_velocity(seq);
  }

  return true;
}

SolverState SolverPipeline::compute_global_state(Registry& registry) {
  // Gather state count and state offsets.
  int offset = 0;
  for (auto& s : registry.get_all<SolverState>()) {
    s.state_offset = offset;
    offset += s.state_num;
  }

  SolverState global_state;
  global_state.state_offset = 0;
  global_state.state_num = offset;
  global_state.curr_state.resize(global_state.state_num);
  global_state.state_velocity.resize(global_state.state_num);

  for (Entity& e : registry.get_all_entities()) {
    auto solver_state = registry.get<SolverState>(e);
    if (!solver_state) {
      continue;
    }

    // Per-entity velocity damping: scale velocities by (1 - damping).
    float damp_factor = 1.0f;
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      damp_factor = 1.0f - cloth_config->damping;
    }

    auto seq = Eigen::seqN(solver_state->state_offset, solver_state->state_num);
    global_state.curr_state(seq) = solver_state->curr_state;
    global_state.state_velocity(seq) =
        damp_factor * solver_state->state_velocity;
  }

  return global_state;
}

Bbox SolverPipeline::compute_scene_bbox(const Eigen::VectorXf& state) {
  int num = state.size();
  auto reshaped = state.reshaped(3, num / 3);
  Eigen::Vector3f min = reshaped.rowwise().minCoeff();
  Eigen::Vector3f max = reshaped.rowwise().maxCoeff();

  return Bbox{min, max};
}

void SolverPipeline::cleanup_collisions(Registry& registry) {
  int i = 0;
  while (i < collisions_.size()) {
    auto& c = collisions_[i];
    if (registry.get_entity(c.entity_handle_a) &&
        registry.get_entity(c.entity_handle_b)) {
      ++i;
      continue;
    }
    // Swap-remove invalid collisions.
    collisions_[i] = collisions_.back();
    collisions_.pop_back();
  }
}

BarrierConstrain SolverPipeline::compute_barrier_constrain(
    const Eigen::VectorXf& state) {
  int state_num = state.size();
  Eigen::VectorXf lhs = Eigen::VectorXf::Zero(state_num);
  Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state_num);

  if (collisions_.empty()) {
    return BarrierConstrain{lhs, rhs};
  }

  for (auto& c : collisions_) {
    // Zero stiffness is not expected currently; kept for future
    // non-distance-barrier update.
    if (c.stiffness == 0.0f) {
      continue;
    }

    for (int i = 0; i < 4; ++i) {
      // If inverse mass is 0, this is either a pinned vertex or an obstacle.
      if (c.inv_mass(i) == 0.0f) {
        continue;
      }

      auto seq = Eigen::seqN(c.offset(i), 3);
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
    }
  }

  return BarrierConstrain{std::move(lhs), std::move(rhs)};
}

void SolverPipeline::enforce_barrier_constrain(
    const BarrierConstrain& barrier_constrain, const Bbox& scene_bbox,
    Eigen::VectorXf& state) const {
  if (collisions_.empty()) {
    return;
  }

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

}  // namespace silk
