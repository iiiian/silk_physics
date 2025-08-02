#include "solver.hpp"

#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/UmfPackSupport>
#include <iostream>  // fix a bug in eigen arpack that miss this include
#include <limits>
#include <memory>
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "collision.hpp"
#include "collision_pipeline.hpp"
#include "ecs.hpp"
#include "eigen_utils.hpp"
#include "object_collider_utils.hpp"
#include "solver_constrain.hpp"
#include "solver_data_utils.hpp"

namespace silk {

const Eigen::VectorXf& Solver::get_solver_state() const { return curr_state_; }

void Solver::clear() {
  init_state_ = {};
  curr_state_ = {};
  prev_state_ = {};
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
  collisions_.clear();
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
  prev_state_.resize(state_num_);
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
  prev_state_ = init_state_;

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
                      Eigen::VectorXf& state) {
  // init rhs is momentum energy term + pin position constrain
  Eigen::VectorXf b = init_rhs;

  // project barrier constrain
  // TODO: avoid hard-coded collision stiffness
  Eigen::SparseMatrix<float> dH(state_num_, state_num_);
  for (auto& c : collisions_) {
    for (int i = 0; i < 4; ++i) {
      dH.coeffRef(c.offset(i), c.offset(i)) = 1e10f;
      dH.coeffRef(c.offset(i) + 1, c.offset(i) + 1) = 1e10f;
      dH.coeffRef(c.offset(i) + 2, c.offset(i) + 2) = 1e10f;

      b(Eigen::seqN(c.offset(i), 3)) = 1e10f * c.position.col(i);
    }
  }

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

bool Solver::step(Registry& registry,
                  const CollisionPipeline& collision_pipeline) {
  init_all_object_collider(registry);
  update_all_obstacle_object_collider(registry);

  // prediction based on linear velocity
  Eigen::VectorXf velocity = (curr_state_ - prev_state_) / dt;
  Eigen::VectorXf acceleration =
      const_acceleration.replicate(state_num_ / 3, 1);
  Eigen::VectorXf init_rhs = (mass_ / (dt * dt)) * curr_state_ +
                             (mass_ / dt) * velocity + mass_ * acceleration;

  // project position constrain from pin groups
  for (Entity& e : registry.get_all_entities()) {
    auto solver_data = registry.get<SolverData>(e);
    auto pin = registry.get<Pin>(e);
    if (solver_data && pin) {
      auto p = pin;

      int offset = solver_data->state_offset;
      for (int i = 0; i < p->index.size(); ++i) {
        init_rhs(Eigen::seqN(offset + 3 * p->index(i), 3)) +=
            p->pin_stiffness * p->position(Eigen::seqN(3 * i, 3));
      }
    }
  }

  prev_state_ = curr_state_;
  Eigen::VectorXf predict_state =
      curr_state_ + dt * velocity + (dt * dt) * acceleration;
  curr_state_ = predict_state;
  // Eigen::VectorXf predict_state = curr_state_;

  collisions_.clear();

  bool has_converge = false;
  float state_diff = 1e6f;
  int iter_count = 0;

  while (!has_converge) {
    while (!has_converge) {
      lg_solve(registry, init_rhs, predict_state);
      // float new_state_diff = (predict_state - curr_state_).cwiseAbs().sum();
      // std::cout << "new state diff " << new_state_diff << " , state diff "
      //           << state_diff << std::endl;
      //
      // has_converge = (new_state_diff < state_diff &&
      //                 (state_diff - new_state_diff) < 0.1f * state_diff);
      // state_diff = new_state_diff;

      has_converge =
          ((predict_state - curr_state_).array().abs() < 1e-3f).all();

      curr_state_ = predict_state;

      std::cout << "Iter " << iter_count << std::endl;
      ++iter_count;
    }

    std::cout << "reach outer loop" << std::endl;

    // update collision
    update_all_physical_object_collider(registry, predict_state, prev_state_);

    std::cout << "outer loop obj collider update fin" << std::endl;

    collisions_ = collision_pipeline.find_collision(
        registry.get_all<ObjectCollider>(), dt);
    std::cout << "get " << collisions_.size() << " collision" << std::endl;

    // ccd line search
    if (!collisions_.empty()) {
      float toi = std::numeric_limits<float>::infinity();
      for (auto& c : collisions_) {
        toi = std::min(toi, c.toi);
      }

      predict_state =
          ccd_walkback * toi * (predict_state - prev_state_) + prev_state_;
      has_converge =
          ((predict_state - curr_state_).array().abs() < 1e-3f).all();

      std::cout << ", toi " << toi << std::endl;
    }
  }

  return true;
}

}  // namespace silk
