#include "solver.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <iostream>  // fix a bug in eigen arpack that miss this include
#include <limits>
#include <memory>
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

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
  mass_ = {};
  H_ = {};
  UHU_ = {};
  U_ = {};
  HX_ = {};
  constrains_.clear();
}

void Solver::reset() {
  curr_state_ = init_state_;
  state_velocity_ = Eigen::VectorXf::Zero(state_num_);
}

bool Solver::init(Registry& registry) {
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
  curr_state_.resize(state_num_);
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
  std::vector<Eigen::Triplet<float>> AA_triplets;
  constrains_.clear();
  for (Entity& e : registry.get_all_entities()) {
    auto data = registry.get<SolverData>(e);

    // vectorize mass
    mass(Eigen::seqN(data->state_offset, data->state_num)) =
        data->mass.replicate(1, 3).reshaped<Eigen::RowMajor>();

    sparse_to_triplets(data->weighted_AA, data->state_offset,
                       data->state_offset, AA_triplets);
    for (auto& constrain : data->constrains) {
      constrain->set_solver_state_offset(data->state_offset);
      constrains_.push_back(constrain.get());
    }
  }

  // compute internal matrices
  mass_ = Eigen::SparseMatrix<float>(state_num_, state_num_);
  mass_ = mass.asDiagonal();

  Eigen::SparseMatrix<float> AA(state_num_, state_num_);
  AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());

  H_ = mass_ / (dt * dt) + AA;

  // TODO: replace arpack with modern solution.
  Eigen::ArpackGeneralizedSelfAdjointEigenSolver<
      Eigen::SparseMatrix<float>,
      Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>>
      eigen_solver;
  r = std::min(r, state_num_);
  eigen_solver.compute(H_, r, "SM");
  if (eigen_solver.info() != Eigen::Success) {
    return false;
  }

  // columns are eigen vectors
  // the dimension of U is nxr
  U_ = eigen_solver.eigenvectors();
  UHU_ = eigen_solver.eigenvalues();
  HX_ = H_ * init_state_;

  return true;
}

bool Solver::lg_solve(Registry& registry, const Eigen::VectorXf& init_rhs,
                      const Eigen::SparseMatrix<float> dH,
                      Eigen::VectorXf& state) {
  Eigen::VectorXf b = init_rhs;

  // project physical constrains
  std::vector<Eigen::VectorXf> buffers(thread_num,
                                       Eigen::VectorXf::Zero(state_num_));
#pragma omp parallel for num_threads(thread_num)
  for (const auto& c : constrains_) {
    Eigen::VectorXf& buffer = buffers[omp_get_thread_num()];
    c->project(state, buffer);
  }
  // merge thread local b back to global b
  for (Eigen::VectorXf& buffer : buffers) {
    b += buffer;
  }

  // subspace solve
  // Eigen::VectorXf subspace_b = U_.transpose() * (b - HX_ - dH * init_state_);
  // Eigen::VectorXf subspace_dx =
  //     (UHU_ + U_.transpose() * dH * U_).householderQr().solve(subspace_b);
  // Eigen::VectorXf subspace_x = init_state_ + U_ * subspace_dx;

  // // iterative global solve
  // Eigen::BiCGSTAB<Eigen::SparseMatrix<float>> iterative_solver;
  // iterative_solver.setMaxIterations(max_iteration);
  // iterative_solver.compute(H_ + dH);
  // state = iterative_solver.solveWithGuess(b, state);
  //
  // // we do not care if iterative solver converges or not because looping
  // // lg_solve is more effective
  // if (iterative_solver.info() != Eigen::Success &&
  //     iterative_solver.info() != Eigen::NoConvergence) {
  //   return false;
  // }
  //
  // std::cout << "iterative solver iter " << iterative_solver.iterations()
  //           << std::endl;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt_solver;
  ldlt_solver.compute(H_ + dH);
  state = ldlt_solver.solve(b);

  if (ldlt_solver.info() != Eigen::Success) {
    return false;
  }

  return true;
}

bool Solver::step(Registry& registry, CollisionPipeline& collision_pipeline) {
  SPDLOG_DEBUG("solver step");

  // TODO: obj collider sub dt update
  init_all_object_collider(registry);
  update_all_obstacle_object_collider(registry);

  Eigen::VectorXf acceleration =
      const_acceleration.replicate(state_num_ / 3, 1);

  // project position constrain from pin groups
  Eigen::VectorXf pin_rhs = Eigen::VectorXf::Zero(state_num_);
  for (Entity& e : registry.get_all_entities()) {
    auto solver_data = registry.get<SolverData>(e);
    auto pin = registry.get<Pin>(e);
    if (solver_data && pin) {
      auto p = pin;

      int offset = solver_data->state_offset;
      for (int i = 0; i < p->index.size(); ++i) {
        pin_rhs(Eigen::seqN(offset + 3 * p->index(i), 3)) +=
            p->pin_stiffness * p->position(Eigen::seqN(3 * i, 3));
      }
    }
  }

  Eigen::VectorXf next_state;
  float remaining_step = 1.0f;

  for (int outer_it = 0; outer_it < max_outer_iteration; ++outer_it) {
    SPDLOG_DEBUG("Outer iter {}", outer_it);

    // prediction based on linear velocity
    next_state = curr_state_ + dt * state_velocity_ + (dt * dt) * acceleration;
    Eigen::VectorXf init_rhs = (mass_ / (dt * dt)) * curr_state_ +
                               (mass_ / dt) * state_velocity_ +
                               mass_ * acceleration + pin_rhs;

    Eigen::VectorXf lg_solution = next_state;
    for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
      SPDLOG_DEBUG("Inner iter {}", inner_it);

      // barrier constrain and projection
      Eigen::SparseMatrix<float> dH(state_num_, state_num_);
      Eigen::VectorXf rhs = init_rhs;
      for (auto& c : collisions_) {
        if (c.stiffness == 0) {
          continue;
        }

        for (int i = 0; i < 4; ++i) {
          // SPDLOG_DEBUG("barrier constrain:");
          if (c.inv_mass(i) == 0.0f) {
            // SPDLOG_DEBUG("inv mass 0, pass");
            continue;
          }

          int offset = c.offset(i);

          // SPDLOG_DEBUG("offset {}, state num {}", offset, state_num_);
          // SPDLOG_DEBUG("original H {} {} {}", H_.coeff(offset, offset),
          //              H_.coeff(offset + 1, offset + 1),
          //              H_.coeff(offset + 2, offset + 2));
          // SPDLOG_DEBUG("init rhs {}",
          //              init_rhs(Eigen::seqN(offset, 3)).transpose());
          // SPDLOG_DEBUG("curr state {}",
          //              curr_state_(Eigen::seqN(offset, 3)).transpose());
          // SPDLOG_DEBUG("reflection state {}",
          // c.reflection.col(i).transpose());

          dH.coeffRef(offset, offset) += c.stiffness;
          dH.coeffRef(offset + 1, offset + 1) += c.stiffness;
          dH.coeffRef(offset + 2, offset + 2) += c.stiffness;

          Eigen::Vector3f position_t0 = curr_state_(Eigen::seqN(offset, 3));
          Eigen::Vector3f reflection =
              position_t0 + c.toi * dt * c.velocity_t0.col(i) +
              (1.0f - c.toi) * dt * c.velocity_t1.col(i);
          rhs(Eigen::seqN(offset, 3)) += c.stiffness * reflection;

          // SPDLOG_DEBUG("toi offset {}", earliest_toi);
          // SPDLOG_DEBUG("reflection state {}", reflection.transpose());
        }
      }

      lg_solve(registry, rhs, dH, next_state);
      if (next_state.array().isNaN().any()) {
        SPDLOG_ERROR("solver explodes");
        exit(1);
        return false;
      }

      if (((next_state - lg_solution).array().abs() < 1e-3f).all()) {
        SPDLOG_DEBUG("lg loop terminate");
        break;
      }
      SPDLOG_DEBUG("lg loop max state diff {}",
                   (next_state - lg_solution).cwiseAbs().maxCoeff());

      lg_solution = next_state;

      // collision stiffness update using partial ccd
      collision_pipeline.update_collision(next_state, curr_state_, collisions_);
    }

    // full collision update
    update_all_physical_object_collider(registry, next_state, curr_state_);
    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), dt);

    // if (collisions.empty()) {
    //   SPDLOG_DEBUG("no collision, terminate outer loop.");
    //   break;
    // }

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
  }

  return true;
}

}  // namespace silk
