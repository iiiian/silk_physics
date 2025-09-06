#include "solver_pipeline.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cmath>
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
  }

  collisions_.clear();
}

void SolverPipeline::reset(Registry& registry) {
  for (auto& d : registry.get_all<ClothDynamicSolverData>()) {
    reset_cloth_dynamic_solver_data(d);
  }

  collisions_.clear();
}

bool SolverPipeline::init(Registry& registry) {
  if (!init_all_cloth_solver_data(registry, dt)) {
    return false;
  }

  return true;
}

bool SolverPipeline::step(Registry& registry,
                          CollisionPipeline& collision_pipeline) {
  SPDLOG_DEBUG("solver step");

  // TODO: obj collider sub dt update
  init_all_object_collider(registry);
  update_all_obstacle_object_collider(registry);
  // collect state and state velocity of all phyiscal entities into one vector
  Eigen::VectorXf state;
  Eigen::VectorXf state_velocity;
  gather_solver_state(registry, state, state_velocity);
  int state_num = state.size();
  // scene bbox is used to estimate the termination criterea of inner loop and
  // the floating point precision of collision pipeline.
  Bbox scene_bbox = compute_scene_bbox(state);
  // clean up collisions involving removed entities.
  cleanup_collisions(registry);
  Eigen::VectorXf acceleration = const_acceleration.replicate(state_num / 3, 1);
  Eigen::VectorXf pin_rhs = gather_pin_constrain(registry, state_num);

  float remaining_step = 1.0f;
  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    Eigen::VectorXf outer_rhs = pin_rhs;
    BarrierConstrain barrier_constrain = compute_barrier_constrain(state);
    // prediction based on linear velocity
    Eigen::VectorXf next_state =
        state + dt * state_velocity + (dt * dt) * acceleration;

    // TODO:: handle error
    init_all_cloth_outer_loop(registry, state, state_velocity, acceleration,
                              barrier_constrain, outer_rhs);

    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      Eigen::VectorXf solution(state_num);
      solve_all_cloth_inner_loop(registry, next_state, outer_rhs, solution);

      if (solution.array().isNaN().any()) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }

      float scene_scale = (scene_bbox.max - scene_bbox.min).norm();
      float threshold = 0.05f * scene_scale;
      if ((solution - next_state).norm() <= threshold) {
        SPDLOG_DEBUG("||dx|| < {}, lg loop terminate", threshold);

        next_state = solution;
        break;
      }

      next_state = solution;
    }

    enforce_barrier_constrain(barrier_constrain, scene_bbox, next_state);

    // full collision update
    update_all_physical_object_collider(registry, next_state, state);
    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), scene_bbox, dt);

    // ccd line search
    float earliest_toi = 1.0f;
    if (!collisions_.empty()) {
      SPDLOG_DEBUG("find {} collisions", collisions_.size());
      for (auto& c : collisions_) {
        earliest_toi = std::min(earliest_toi, c.toi);
      }
      earliest_toi *= 0.8f;
      SPDLOG_DEBUG("earliest toi {}", earliest_toi);
    }

    state_velocity = (next_state - state) / dt;

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      for (auto& c : collisions_) {
        c.toi -= remaining_step;
      }
      state += remaining_step * (next_state - state);
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", earliest_toi);
    next_state = earliest_toi * (next_state - state) + state;
    state = next_state;
    remaining_step -= earliest_toi;
    for (auto& c : collisions_) {
      c.toi -= earliest_toi;
    }
  }

  return true;
}

void SolverPipeline::gather_solver_state(Registry& registry,
                                         Eigen::VectorXf& state,
                                         Eigen::VectorXf& state_velocity) {
  int offset = 0;
  // gather state num and state offset
  for (auto& d : registry.get_all<ClothDynamicSolverData>()) {
    d.state_offset = offset;
    offset += d.state_num;
  }

  state.resize(offset);
  state_velocity.resize(offset);

  for (auto& d : registry.get_all<ClothDynamicSolverData>()) {
    auto seq = Eigen::seqN(d.state_offset, d.state_num);
    state(seq) = d.curr_state;
    state_velocity(seq) = d.state_velocity;
  }
}

Bbox SolverPipeline::compute_scene_bbox(const Eigen::VectorXf& state) {
  int num = state.size();
  auto reshaped = state.reshaped(3, num / 3);
  Eigen::Vector3f min = reshaped.rowwise().minCoeff();
  Eigen::Vector3f max = reshaped.rowwise().maxCoeff();

  return Bbox{min, max};
}

void SolverPipeline::cleanup_collisions(Registry& registry) {
  for (int i = 0; i < collisions_.size(); ++i) {
    auto& c = collisions_[i];
    if (registry.get_entity(c.entity_a) && registry.get_entity(c.entity_b)) {
      continue;
    }
    collisions_[i] = collisions_.back();
    collisions_.pop_back();
  }
}

Eigen::VectorXf SolverPipeline::gather_pin_constrain(Registry& registry,
                                                     int state_num) {
  Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state_num);

  for (Entity& e : registry.get_all_entities()) {
    auto data = registry.get<ClothDynamicSolverData>(e);
    auto pin = registry.get<Pin>(e);
    if (data && pin) {
      int offset = data->state_offset;
      for (int i = 0; i < pin->index.size(); ++i) {
        rhs(Eigen::seqN(offset + 3 * pin->index(i), 3)) +=
            pin->pin_stiffness * pin->position(Eigen::seqN(3 * i, 3));
      }
    }
  }

  return rhs;
}

BarrierConstrain SolverPipeline::compute_barrier_constrain(
    const Eigen::VectorXf& state) {
  int state_num = state.size();
  Eigen::VectorXf lhs = Eigen::VectorXf::Zero(state_num);
  Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state_num);

  if (collisions_.empty()) {
    return BarrierConstrain{lhs, rhs};
  }

  // target state is used as a temp buffer for rhs change
  rhs = Eigen::VectorXf::Zero(state_num);
  for (auto& c : collisions_) {
    if (c.stiffness == 0.0f) {
      continue;
    }

    for (int i = 0; i < 4; ++i) {
      if (c.inv_mass(i) == 0.0f) {
        continue;
      }

      int offset = c.offset(i);

      Eigen::Vector3f position_t0 = state(Eigen::seqN(offset, 3));
      Eigen::Vector3f reflection;
      if (c.use_small_ms) {
        reflection = position_t0 + c.velocity_t1.col(i);
      } else {
        reflection = position_t0 + c.toi * c.velocity_t0.col(i) +
                     (1.0f - c.toi) * c.velocity_t1.col(i);
      }

      lhs(Eigen::seqN(offset, 3)) += c.stiffness * Eigen::Vector3f::Ones();
      rhs(Eigen::seqN(offset, 3)) += c.stiffness * reflection;
    }
  }

  return BarrierConstrain{std::move(lhs), std::move(rhs)};
}

void SolverPipeline::enforce_barrier_constrain(
    const BarrierConstrain& barrier_constrain, const Bbox& scene_bbox,
    Eigen::VectorXf& state) {
  if (collisions_.empty()) {
    return;
  }

  auto& b = barrier_constrain;
  int state_num = state.size();
  float scene_scale = (scene_bbox.max - scene_bbox.min).norm();
  float threshold = 1e-5f * scene_scale;

  for (int i = 0; i < state_num; ++i) {
    if (b.lhs(i) == 0.0f) {
      continue;
    }

    float target = b.rhs(i) - b.lhs(i);
    float abs_diff = std::abs(target - state(i));
    if (abs_diff > threshold) {
      SPDLOG_DEBUG("barrier constrain fails. abs diff {} > {}", abs_diff,
                   threshold);
    }

    state(i) = target;
  }
}

}  // namespace silk
