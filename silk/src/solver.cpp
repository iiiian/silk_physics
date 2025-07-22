#include "solver.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>  // fix a bug in eigen arpack that miss this include
#include <limits>
#include <memory>
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "collision.hpp"
#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "make_obstacle.hpp"
#include "make_solver_data.hpp"
#include "solver_constrain.hpp"
#include "update_obstacle.hpp"

namespace silk {

const Eigen::VectorXf& Solver::get_solver_state() const { return curr_state_; }

void Solver::clear() {
  init_state_ = {};
  curr_state_ = {};
  prev_state_ = {};
  prev_velocity = {};
  mass_ = {};
  H_ = {};
  UHU_ = {};
  U_ = {};
  HX_ = {};
  constrains_.clear();
  collisions_.clear();
}

void Solver::reset() {
  curr_state_ = init_state_;
  prev_state_ = init_state_;
  prev_velocity = Eigen::VectorXf::Zero(state_num_);
  collisions_.clear();
}

bool Solver::init(Registry& registry) {
  make_all_solver_data(registry);
  make_all_obstacles(registry);

  // count total state num
  state_num_ = 0;
  for (Entity& e : registry.entity.data()) {
    auto solver_data = ECS_GET(registry, e, solver_data);
    if (solver_data) {
      state_num_ += solver_data->state_num;
    }
  }

  // initialize state vector
  init_state_.resize(state_num_);
  curr_state_.resize(state_num_);
  prev_state_.resize(state_num_);
  prev_velocity = Eigen::VectorXf::Zero(state_num_);
  for (Entity& e : registry.entity.data()) {
    auto solver_data = ECS_GET(registry, e, solver_data);
    auto tri_mesh = ECS_GET(registry, e, tri_mesh);

    if (solver_data && tri_mesh) {
      init_state_(
          Eigen::seqN(solver_data->state_offset, solver_data->state_num)) =
          tri_mesh->V.reshaped<Eigen::RowMajor>();
      continue;
    }
  }
  curr_state_ = init_state_;
  prev_state_ = init_state_;

  // collect solver data
  std::vector<Eigen::Triplet<float>> mass_triplets;
  std::vector<Eigen::Triplet<float>> AA_triplets;
  constrains_.clear();
  for (Entity& e : registry.entity.data()) {
    auto solver_data = ECS_GET(registry, e, solver_data);
    if (solver_data) {
      // vectorize vertex mass
      auto& m = solver_data->mass;
      for (int i = 0; i < solver_data->state_num; ++i) {
        int offset = solver_data->state_offset + 3 * i;
        mass_triplets.emplace_back(offset, offset, m(i));
        mass_triplets.emplace_back(offset + 1, offset + 1, m(i));
        mass_triplets.emplace_back(offset + 2, offset + 2, m(i));
      }

      AA_triplets.insert(AA_triplets.end(), solver_data->weighted_AA.begin(),
                         solver_data->weighted_AA.end());
      constrains_.insert(constrains_.end(), solver_data->constrains.begin(),
                         solver_data->constrains.end());

      solver_data->mass = {};
      solver_data->weighted_AA = {};
      solver_data->constrains = {};
    }
  }

  // compute internal matrices

  mass_.resize(state_num_, state_num_);
  mass_.setFromTriplets(mass_triplets.begin(), mass_triplets.end());

  Eigen::SparseMatrix<float> AA(state_num_, state_num_);
  AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());

  H_ = (mass_ / (dt_ * dt_) + AA);

  // TODO: replace arpack with modern solution.
  Eigen::ArpackGeneralizedSelfAdjointEigenSolver<
      Eigen::SparseMatrix<float>,
      Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>>
      eigen_solver;
  r_ = std::min(r_, state_num_);
  eigen_solver.compute(H_, r_, "SM");
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

bool Solver::lg_solve(Registry& registry, Eigen::VectorXf& predict_state) {
  // momentum energy term
  Eigen::VectorXf b = (mass_ / dt_ / dt_) * predict_state +
                      (mass_ / dt_) * (predict_state - prev_state_) +
                      mass_ * const_acceleration.replicate(state_num_, 1);
  Eigen::SparseMatrix<float> A = H_;

  // porject barrier constrain
  // TODO: avoid hard-coded collision stiffness
  for (auto& c : collisions_) {
    for (int i = 0; i < 4; ++i) {
      A.coeffRef(c.offset(i), c.offset(i)) = 1e6f * c.position(0, i);
      A.coeffRef(c.offset(i) + 1, c.offset(i) + 1) = 1e6f * c.position(1, i);
      A.coeffRef(c.offset(i) + 2, c.offset(i) + 2) = 1e6f * c.position(2, i);
      b(Eigen::seqN(c.offset(i), 3)) = 1e6f * c.position.col(i);
    }
  }

  // project position constrain from pinned groups
  for (Entity& e : registry.entity.data()) {
    auto solver_data = ECS_GET(registry, e, solver_data);
    auto pinned_group = ECS_GET(registry, e, pinned_group);
    if (solver_data && pinned_group) {
      auto p = pinned_group;

      int offset = solver_data->state_offset;
      for (int i = 0; i < p->pinnned_index.size(); ++i) {
        b(Eigen::seqN(offset + 3 * p->pinnned_index(i), 3)) +=
            p->pinned_value(Eigen::seqN(3 * i, 3));
      }
    }
  }

  // project other constrains
  std::vector<Eigen::VectorXf> buffers(thread_num,
                                       Eigen::VectorXf::Zero(state_num_));
#pragma omp parallel for num_threads(thread_num)
  for (const auto& c : constrains_) {
    Eigen::VectorXf& buffer = buffers[omp_get_thread_num()];
    c->project(curr_state_, buffer);
  }
  // merge thread local b back to global b
  for (Eigen::VectorXf& buffer : buffers) {
    b += buffer;
  }

  // subspace solve
  Eigen::VectorXf subspace_b = U_.transpose() * (b - HX_);
  Eigen::VectorXf q;
  if (collisions_.empty()) {
    q = subspace_b.array() / UHU_.array();
  } else {
    q = (U_.transpose() * A * U_).householderQr().solve(subspace_b);
  }
  Eigen::VectorXf subspace_sol = init_state_ + U_ * q;

  // iterative global solve
  Eigen::BiCGSTAB<Eigen::SparseMatrix<float>> iterative_solver;
  iterative_solver.setMaxIterations(max_iteration);
  iterative_solver.compute(H_);
  predict_state = iterative_solver.solveWithGuess(b, subspace_sol);

  // we do not care if iterative solver converges or not because looping
  // lg_solve is more effective
  if (iterative_solver.info() != Eigen::Success &&
      iterative_solver.info() != Eigen::NoConvergence) {
    return false;
  }

  return true;
}

bool Solver::step(Registry& registry,
                  const CollisionPipeline& collision_pipeline) {
  // prediction based on linear velocity
  Eigen::VectorXf velocity = (curr_state_ - prev_state_) / dt_;
  Eigen::VectorXf acceleration = (velocity - prev_velocity) / dt_ +
                                 const_acceleration.replicate(state_num_, 1);
  Eigen::VectorXf predict_state =
      curr_state_ + dt_ * velocity + dt_ * dt_ * acceleration;
  prev_velocity = velocity;
  prev_state_ = curr_state_;
  curr_state_ = predict_state;

  float state_diff = std::numeric_limits<float>::infinity();
  while (state_diff > 1e-3f) {
    while (state_diff > 5e-2f) {
      lg_solve(registry, predict_state);
      state_diff = (predict_state - curr_state_).cwiseAbs().sum();
      curr_state_ = predict_state;
    }

    // update collision
    update_all_obstacles(registry, predict_state, prev_state_);
    collisions_ =
        collision_pipeline.find_collision(registry.obstacle.data(), dt_);

    // ccd line search
    if (!collisions_.empty()) {
      float toi = std::numeric_limits<float>::infinity();
      for (auto& c : collisions_) {
        toi = std::min(toi, c.toi);
      }

      curr_state_ +=
          ccd_line_search_walkback * toi * (predict_state - curr_state_);
      state_diff = (predict_state - curr_state_).cwiseAbs().sum();
    }
  }

  return true;
}

}  // namespace silk
