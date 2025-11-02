#include "pipeline.hpp"

#include <Eigen/Core>
#include <silk/silk.hpp>
#include <vector>

#include "cloth_solver_utils.hpp"
#include "cloth_topology.hpp"
#include "collision/cpu/bbox.hpp"
#include "collision/cpu/collision.hpp"
#include "collision/cpu/object_collider.hpp"
#include "ecs.hpp"
#include "logger.hpp"
#include "mesh.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "solver/cpu/cloth_solver_context.hpp"
#include "solver/cpu/cloth_solver_utils.hpp"
#include "solver/cpu/obstacle_solver_utils.hpp"
#include "solver/gpu/gpu_cloth_solver_context.hpp"

namespace silk {
namespace gpu {

void GpuSolverPipeline::clear(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    registry.remove<ClothTopology>(e);
    registry.remove<CpuClothSolverContext>(e);
    registry.remove<ObjectState>(e);
    registry.remove<CpuObjectCollider>(e);
  }

  // Clear GPU contexts
  gpu_contexts_.clear();
}

void GpuSolverPipeline::reset(Registry& registry) {
  batch_reset_cloth_simulation(registry);
  batch_reset_obstacle_simulation(registry);

  // GPU contexts will be recreated on next init
  gpu_contexts_.clear();
}

bool GpuSolverPipeline::step(Registry& registry) {
  SPDLOG_DEBUG("GPU solver step");

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
  std::vector<Collision> collisions;

  // Scene bbox is used to estimate the termination criteria of the inner loop
  // and the floating-point precision of the collision pipeline.
  Bbox scene_bbox = compute_scene_bbox(curr_state);

  // Compute step invariant rhs.
  Eigen::VectorXf init_rhs = Eigen::VectorXf::Zero(state_num);
  batch_compute_cloth_invariant_rhs(registry, init_rhs);

  // Expand per-vertex acceleration (XYZ per vertex) to the packed state vector.
  Eigen::VectorXf acceleration = const_acceleration.replicate(state_num / 3, 1);

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
      SPDLOG_DEBUG("Inner iter {} (GPU)", inner_it);

      Eigen::VectorXf solution(state_num);

      // --- GPU-ACCELERATED INNER LOOP ---
      // Process each cloth entity with GPU kernels
      int entity_idx = 0;
      for (Entity& e : registry.get_all_entities()) {
        auto config = registry.get<ClothConfig>(e);
        auto mesh = registry.get<TriMesh>(e);
        auto topology = registry.get<ClothTopology>(e);
        auto cpu_context = registry.get<CpuClothSolverContext>(e);
        auto state = registry.get<ObjectState>(e);

        if (!(config && mesh && topology && cpu_context && state)) {
          continue;
        }

        // Get GPU context for this entity
        if (entity_idx >= gpu_contexts_.size() ||
            !gpu_contexts_[entity_idx]) {
          SPDLOG_ERROR("GPU context not initialized for entity {}", entity_idx);
          return false;
        }

        auto& gpu_ctx = *gpu_contexts_[entity_idx];
        entity_idx++;

        auto seq = Eigen::seqN(state->state_offset, state->state_num);

        // Use GPU-accelerated inner loop
        if (!compute_cloth_inner_loop_gpu(*config, mesh->F, *topology,
                                          *cpu_context, gpu_ctx,
                                          next_state(seq), outer_rhs(seq),
                                          solution(seq))) {
          SPDLOG_ERROR("GPU inner loop failed for entity");
          return false;
        }
      }

      if (!solution.allFinite()) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }

      float scene_scale = (scene_bbox.max - scene_bbox.min).norm();
      assert(scene_scale != 0);
      float threshold = 0.05f * scene_scale;
      if ((solution - next_state).norm() <= threshold) {
        SPDLOG_DEBUG("||dx|| < {}, inner loop terminate", threshold);
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

    // Full collision update.
    for (Entity& e : registry.get_all_entities()) {
      auto config = registry.get<CollisionConfig>(e);
      auto state = registry.get<ObjectState>(e);
      auto collider = registry.get<CpuObjectCollider>(e);
      if (!(config && state && collider)) {
        continue;
      }
      collider->update(*config, *state, next_state, curr_state);
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

    state_velocity = (next_state - curr_state) / dt;

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      curr_state += remaining_step * (next_state - curr_state);
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", earliest_toi);
    curr_state = earliest_toi * (next_state - curr_state) + curr_state;
    remaining_step -= earliest_toi;
    for (auto& c : collisions) {
      c.toi -= earliest_toi;
    }
  }

  // Write solution back to registry
  for (auto& state : registry.get_all<ObjectState>()) {
    auto seq = Eigen::seqN(state.state_offset, state.state_num);
    state.curr_state = curr_state(seq);
    state.state_velocity = state_velocity(seq);
  }

  return true;
}

// Lazily init all entities and create GPU contexts.
bool GpuSolverPipeline::init(Registry& registry, ObjectState& global_state) {
  int state_num = 0;
  for (Entity& e : registry.get_all_entities()) {
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

  for (Entity& e : registry.get_all_entities()) {
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

  // --- Initialize GPU contexts for all cloth entities ---
  if (gpu_contexts_.empty()) {
    SPDLOG_INFO("Initializing GPU contexts for cloth entities");
    int entity_count = 0;
    for (Entity& e : registry.get_all_entities()) {
      auto cloth_config = registry.get<ClothConfig>(e);
      if (cloth_config) {
        entity_count++;
      }
    }

    gpu_contexts_.resize(entity_count);

    int entity_idx = 0;
    for (Entity& e : registry.get_all_entities()) {
      auto cloth_config = registry.get<ClothConfig>(e);
      if (cloth_config) {
        if (!init_gpu_context_for_entity(e, registry)) {
          SPDLOG_ERROR("Failed to initialize GPU context for entity {}",
                       entity_idx);
          return false;
        }
        entity_idx++;
      }
    }

    SPDLOG_INFO("GPU contexts initialized for {} cloth entities", entity_count);
  }

  return true;
}

bool GpuSolverPipeline::init_gpu_context_for_entity(Entity& entity,
                                                     Registry& registry) {
  auto config = registry.get<ClothConfig>(entity);
  auto mesh = registry.get<TriMesh>(entity);
  auto topology = registry.get<ClothTopology>(entity);

  if (!(config && mesh && topology)) {
    return false;
  }

  // Find entity index
  int entity_idx = 0;
  for (Entity& e : registry.get_all_entities()) {
    auto cloth_config = registry.get<ClothConfig>(e);
    if (cloth_config) {
      if (e.self == entity.self) {
        break;
      }
      entity_idx++;
    }
  }

  auto gpu_ctx =
      GpuClothSolverContext::create(*config, *topology, mesh->F, dt);

  if (!gpu_ctx) {
    SPDLOG_ERROR("Failed to create GPU context");
    return false;
  }

  gpu_contexts_[entity_idx] =
      std::make_unique<GpuClothSolverContext>(std::move(*gpu_ctx));

  SPDLOG_INFO("GPU context created for entity {}: {} faces, {} vertices",
              entity_idx, mesh->F.rows(), mesh->V.rows());

  return true;
}

Bbox GpuSolverPipeline::compute_scene_bbox(const Eigen::VectorXf& state) {
  int num = state.size();
  auto reshaped = state.reshaped(3, num / 3);
  Eigen::Vector3f min = reshaped.rowwise().minCoeff();
  Eigen::Vector3f max = reshaped.rowwise().maxCoeff();

  return Bbox{min, max};
}

BarrierConstrain GpuSolverPipeline::compute_barrier_constrain(
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

void GpuSolverPipeline::enforce_barrier_constrain(
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

}  // namespace gpu
}  // namespace silk
