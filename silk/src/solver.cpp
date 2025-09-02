#include "solver.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "cholmod_utils.hpp"
#include "collision.hpp"
#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "eigen_utils.hpp"
#include "logger.hpp"
#include "object_collider_utils.hpp"
#include "solver_constrain.hpp"
#include "solver_data_utils.hpp"

namespace silk {

const Eigen::VectorXf& Solver::get_solver_state() const { return curr_state_; }

void Solver::clear() {
  init_state_ = {};
  curr_state_ = {};
  state_velocity_ = {};
  H_ = {};
  L_ = {};
  mass_ = {};
  constrains_.clear();
  collisions_.clear();
  barrier_target_state_ = {};
}

void Solver::reset() {
  curr_state_ = init_state_;
  state_velocity_ = Eigen::VectorXf::Zero(state_num_);
  collisions_.clear();
  barrier_target_state_ = {};
}

bool Solver::init(Registry& registry) {
  clear();
  init_all_solver_data(registry);

  // count total state num
  state_num_ = 0;
  for (Entity& e : registry.get_all_entities()) {
    auto solver_data = registry.get<SolverData>(e);
    if (solver_data) {
      state_num_ += solver_data->state_num;
    }
  }

  // initialize state vector
  init_state_.resize(state_num_);
  for (Entity& e : registry.get_all_entities()) {
    auto solver_data = registry.get<SolverData>(e);
    auto tri_mesh = registry.get<TriMesh>(e);

    if (solver_data && tri_mesh) {
      init_state_(
          Eigen::seqN(solver_data->state_offset, solver_data->state_num)) =
          tri_mesh->V.reshaped<Eigen::RowMajor>();
      continue;
    }
  }
  curr_state_ = init_state_;
  state_velocity_ = Eigen::VectorXf::Zero(state_num_);

  // collect solver data
  Eigen::VectorXf mass(state_num_);
  std::vector<Eigen::Triplet<float>> H_triplets;
  constrains_.clear();
  for (Entity& e : registry.get_all_entities()) {
    auto data = registry.get<SolverData>(e);

    // vectorize mass
    mass(Eigen::seqN(data->state_offset, data->state_num)) =
        data->mass.replicate(1, 3).reshaped<Eigen::RowMajor>();

    append_triplets_from_sparse(data->weighted_AA, data->state_offset,
                                data->state_offset, H_triplets,
                                SymmetricStatus::UpperTriangular);

    for (auto& constrain : data->constrains) {
      constrain->set_solver_state_offset(data->state_offset);
      constrains_.push_back(constrain.get());
    }
  }

  mass_ = Eigen::SparseMatrix<float>(state_num_, state_num_);
  mass_ = mass.asDiagonal();

  H_ = Eigen::SparseMatrix<float>{state_num_, state_num_};
  H_.setFromTriplets(H_triplets.begin(), H_triplets.end());
  H_ += mass_ / (dt * dt);

  cholmod_sparse H_view = cholmod_raii::make_cholmod_sparse_view(
      H_, SymmetricStatus::UpperTriangular);

  L_ = cholmod_analyze(&H_view, cholmod_raii::global_common);
  assert(!L_.is_empty());
  cholmod_factorize(&H_view, L_, cholmod_raii::global_common);

  return true;
}

void Solver::damp_state_velocity(Registry& registry) {
  for (const auto& d : registry.get_all<SolverData>()) {
    float decay = 1.0f - d.damping;
    state_velocity_(Eigen::seqN(d.state_offset, d.state_num)) *= decay;
  }
}

void Solver::compute_pin_constrain(Registry& registry, Eigen::VectorXf& rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto solver_data = registry.get<SolverData>(e);
    auto pin = registry.get<Pin>(e);
    if (solver_data && pin) {
      auto p = pin;

      int offset = solver_data->state_offset;
      for (int i = 0; i < p->index.size(); ++i) {
        rhs(Eigen::seqN(offset + 3 * p->index(i), 3)) +=
            p->pin_stiffness * p->position(Eigen::seqN(3 * i, 3));
      }
    }
  }
}

void Solver::compute_physical_constrain(const Eigen::VectorXf& state,
                                        Eigen::VectorXf& rhs) {
  std::vector<Eigen::VectorXf> buffers(thread_num,
                                       Eigen::VectorXf::Zero(state_num_));
#pragma omp parallel for num_threads(thread_num)
  for (const auto& c : constrains_) {
    Eigen::VectorXf& buffer = buffers[omp_get_thread_num()];
    c->project(state, buffer);
  }
  // merge thread local b back to global b
  for (Eigen::VectorXf& buffer : buffers) {
    rhs += buffer;
  }
}

void Solver::compute_barrier_constrain(Eigen::VectorXf& rhs,
                                       cholmod_raii::CholmodFactor& LC) {
  assert(!collisions_.empty());

  Eigen::VectorXf stiffness_sum = Eigen::VectorXf::Zero(state_num_);
  // target state is used as a temp buffer for rhs change
  barrier_target_state_ = Eigen::VectorXf::Zero(state_num_);
  for (auto& c : collisions_) {
    if (c.stiffness == 0.0f) {
      continue;
    }

    for (int i = 0; i < 4; ++i) {
      if (c.inv_mass(i) == 0.0f) {
        continue;
      }

      int offset = c.offset(i);

      Eigen::Vector3f position_t0 = curr_state_(Eigen::seqN(offset, 3));
      Eigen::Vector3f reflection;
      if (c.use_small_ms) {
        reflection = position_t0 + c.velocity_t1.col(i);
      } else {
        reflection = position_t0 + c.toi * c.velocity_t0.col(i) +
                     (1.0f - c.toi) * c.velocity_t1.col(i);
      }
      barrier_target_state_(Eigen::seqN(offset, 3)) += c.stiffness * reflection;

      stiffness_sum(offset) += c.stiffness;
      stiffness_sum(offset + 1) += c.stiffness;
      stiffness_sum(offset + 2) += c.stiffness;
    }
  }

  rhs += barrier_target_state_;

  Eigen::SparseMatrix<float> C{state_num_, state_num_};
  for (int i = 0; i < state_num_; ++i) {
    if (barrier_target_state_(i) == 0.0f) {
      // indicating there's no barrier constrain for state(i)
      barrier_target_state_(i) = std::numeric_limits<float>::max();
      continue;
    }
    barrier_target_state_(i) /= stiffness_sum(i);
    C.coeffRef(i, i) = stiffness_sum(i);
  }
  // we do sqrt for cholmod updown.
  C = C.cwiseSqrt();

  cholmod_sparse C_view = cholmod_raii::make_cholmod_sparse_view(C);

  int32_t* rset = static_cast<int32_t*>(L_.raw()->Perm);
  int64_t rset_num = static_cast<int64_t>(L_.raw()->n);
  cholmod_raii::CholmodSparse C_perm = cholmod_submatrix(
      &C_view, rset, rset_num, nullptr, -1, 1, 1, cholmod_raii::global_common);

  LC = L_;
  cholmod_updown(1, C_perm, LC, cholmod_raii::global_common);
}

void Solver::enforce_barrier_constrain(Eigen::VectorXf& state) {
  assert(!collisions_.empty());

  float scene_scale = (scene_bbox_.max - scene_bbox_.min).norm();
  float threshold = 1e-5f * scene_scale;

  for (int i = 0; i < state_num_; ++i) {
    if (barrier_target_state_(i) == std::numeric_limits<float>::max()) {
      continue;
    }

    float target = barrier_target_state_(i);
    float abs_diff = std::abs(target - state(i));
    if (abs_diff > threshold) {
      SPDLOG_DEBUG("barrier constrain fails. abs diff {} > {}", abs_diff,
                   threshold);
    }

    state(i) = target;
  }
}

bool Solver::global_solve(Eigen::VectorXf& rhs, cholmod_raii::CholmodFactor& L,
                          Eigen::VectorXf& out) {
  cholmod_dense rhs_view = cholmod_raii::make_cholmod_dense_vector_view(rhs);
  cholmod_raii::CholmodDense sol =
      cholmod_solve(CHOLMOD_A, L, &rhs_view, cholmod_raii::global_common);
  assert(!sol.is_empty());

  out = cholmod_raii::make_eigen_dense_vector_view(sol);

  return true;
}

bool Solver::step(Registry& registry, CollisionPipeline& collision_pipeline) {
  SPDLOG_DEBUG("solver step");

  // TODO: obj collider sub dt update
  init_all_object_collider(registry);
  update_all_obstacle_object_collider(registry);

  scene_bbox_.min =
      curr_state_.reshaped(3, state_num_ / 3).rowwise().minCoeff();
  scene_bbox_.max =
      curr_state_.reshaped(3, state_num_ / 3).rowwise().maxCoeff();

  Eigen::VectorXf acceleration =
      const_acceleration.replicate(state_num_ / 3, 1);

  Eigen::VectorXf pin_rhs = Eigen::VectorXf::Zero(state_num_);
  compute_pin_constrain(registry, pin_rhs);

  Eigen::VectorXf next_state;
  float remaining_step = 1.0f;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    Eigen::VectorXf outer_rhs = (mass_ / (dt * dt)) * curr_state_ +
                                (mass_ / dt) * state_velocity_ +
                                mass_ * acceleration + pin_rhs;

    cholmod_raii::CholmodFactor LC;
    if (!collisions_.empty()) {
      compute_barrier_constrain(outer_rhs, LC);
    }

    // prediction based on linear velocity
    damp_state_velocity(registry);
    next_state = curr_state_ + dt * state_velocity_ + (dt * dt) * acceleration;

    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      Eigen::VectorXf inner_rhs = outer_rhs;
      compute_physical_constrain(next_state, inner_rhs);

      Eigen::VectorXf solution;
      if (collisions_.empty()) {
        global_solve(inner_rhs, L_, solution);
      } else {
        global_solve(inner_rhs, LC, solution);
      }

      if (solution.array().isNaN().any()) {
        SPDLOG_ERROR("solver explodes");
        return false;
      }

      float scene_scale = (scene_bbox_.max - scene_bbox_.min).norm();
      float threshold = 0.05f * scene_scale;
      if ((solution - next_state).norm() <= threshold) {
        SPDLOG_DEBUG("||dx|| < {}, lg loop terminate", threshold);

        next_state = solution;
        break;
      }

      next_state = solution;
    }

    if (!collisions_.empty()) {
      enforce_barrier_constrain(next_state);
    }

    // full collision update
    update_all_physical_object_collider(registry, next_state, curr_state_);
    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), scene_bbox_, dt);

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

    state_velocity_ = (next_state - curr_state_) / dt;

    if (earliest_toi >= remaining_step) {
      SPDLOG_DEBUG(
          "earliest toi  {} >= remaining step {}. terminate outer loop.",
          earliest_toi, remaining_step);
      for (auto& c : collisions_) {
        c.toi -= remaining_step;
      }
      curr_state_ += remaining_step * (next_state - curr_state_);
      break;
    }

    SPDLOG_DEBUG("CCD rollback to toi {}", earliest_toi);
    next_state = earliest_toi * (next_state - curr_state_) + curr_state_;
    curr_state_ = next_state;
    remaining_step -= earliest_toi;
    for (auto& c : collisions_) {
      c.toi -= earliest_toi;
    }
  }

  return true;
}

}  // namespace silk
