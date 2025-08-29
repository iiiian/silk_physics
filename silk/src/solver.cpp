#include "solver.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cmath>
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
}

void Solver::reset() {
  curr_state_ = init_state_;
  state_velocity_ = Eigen::VectorXf::Zero(state_num_);
  collisions_.clear();
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

  // Eigen::SparseMatrix<double> tmp = H_.cast<double>();
  cholmod_sparse H_view = cholmod_raii::make_cholmod_sparse_view(
      H_, SymmetricStatus::UpperTriangular);

  L_ = cholmod_analyze(&H_view, cholmod_raii::global_common);
  assert(!L_.is_empty());
  // cholmod_print_factor(L_, "L", cholmod_raii::global_common);
  cholmod_factorize(&H_view, L_, cholmod_raii::global_common);
  // cholmod_print_factor(L_, "L", cholmod_raii::global_common);

  return true;
}

void Solver::update_rhs_for_pin(Registry& registry, Eigen::VectorXf& rhs) {
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

void Solver::update_rhs_for_physics(const Eigen::VectorXf& state,
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

cholmod_raii::CholmodFactor Solver::update_factor_and_rhs_for_collision(
    Eigen::VectorXf& rhs) {
  if (collisions_.empty()) {
    return nullptr;
  }

  std::vector<Eigen::Triplet<float>> C_triplets;
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
      Eigen::Vector3f reflection = position_t0 +
                                   c.toi * dt * c.velocity_t0.col(i) +
                                   (1.0f - c.toi) * dt * c.velocity_t1.col(i);
      rhs(Eigen::seqN(offset, 3)) += c.stiffness * reflection;

      C_triplets.emplace_back(offset, offset, c.stiffness);
      C_triplets.emplace_back(offset + 1, offset + 1, c.stiffness);
      C_triplets.emplace_back(offset + 2, offset + 2, c.stiffness);
    }
  }

  Eigen::SparseMatrix<float> C{state_num_, state_num_};
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());
  C = C.cwiseSqrt();

  cholmod_sparse C_view = cholmod_raii::make_cholmod_sparse_view(C);

  int32_t* rset = static_cast<int32_t*>(L_.raw()->Perm);
  int64_t rset_num = static_cast<int64_t>(L_.raw()->n);
  cholmod_raii::CholmodSparse C_perm = cholmod_submatrix(
      &C_view, rset, rset_num, nullptr, -1, 1, 1, cholmod_raii::global_common);

  cholmod_raii::CholmodFactor LC = L_;
  cholmod_updown(1, C_perm, LC, cholmod_raii::global_common);

  return LC;
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

  Eigen::VectorXf acceleration =
      const_acceleration.replicate(state_num_ / 3, 1);

  Eigen::VectorXf pin_rhs = Eigen::VectorXf::Zero(state_num_);
  update_rhs_for_pin(registry, pin_rhs);

  Eigen::VectorXf next_state;
  float remaining_step = 1.0f;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    Eigen::VectorXf outer_rhs = (mass_ / (dt * dt)) * curr_state_ +
                                (mass_ / dt) * state_velocity_ +
                                mass_ * acceleration + pin_rhs;
    cholmod_raii::CholmodFactor LC =
        update_factor_and_rhs_for_collision(outer_rhs);

    // prediction based on linear velocity
    next_state = curr_state_ + dt * state_velocity_ + (dt * dt) * acceleration;

    Eigen::VectorXf lg_solution = next_state;
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      Eigen::VectorXf inner_rhs = outer_rhs;
      update_rhs_for_physics(next_state, inner_rhs);

      if (collisions_.empty()) {
        global_solve(inner_rhs, L_, next_state);
      } else {
        global_solve(inner_rhs, LC, next_state);
      }

      if (next_state.array().isNaN().any()) {
        SPDLOG_ERROR("solver explodes");
        exit(1);
        return false;
      }

      if (((next_state - lg_solution).array().abs() < 1e-3f).all()) {
        SPDLOG_DEBUG("lg loop terminate");
        break;
      }

      lg_solution = next_state;

      // collision stiffness update using partial ccd
      collision_pipeline.update_collision(next_state, curr_state_, collisions_);
    }

    // full collision update
    update_all_physical_object_collider(registry, next_state, curr_state_);
    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), dt);

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
