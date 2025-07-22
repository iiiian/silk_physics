#include <Eigen/Core>
#include <vector>

#include "ecs.hpp"
#include "solver.hpp"

namespace silk {

std::string to_string(Result result) {
  switch (result) {
    case Result::Success: {
      return "Success";
    }
    case Result::InvalidTimeStep: {
      return "InvalidTimeStep";
    }
    case Result::InvalidLowFreqModeNum: {
      return "InvalidLowFreqModeNum";
    }
    case Result::InvalidConfig: {
      return "InvalidConfig";
    }
    case Result::TooManyBody: {
      return "TooManyBody";
    }
    case Result::InvalidHandle: {
      return "InvalidHandle";
    }
    case Result::IncorrectPositionConstrainLength: {
      return "IncorrectPositionConstrainLength";
    }
    case Result::IncorrentOutputPositionLength: {
      return "IncorrentOutputPositionLength";
    }
    case Result::EigenDecompositionfail: {
      return "EigenDecompositionfail";
    }
    case Result::IterativeSolverInitFail: {
      return "IterativeSolverInitFail";
    }
    case Result::NeedInitSolverBeforeSolve: {
      return "NeedInitSolverBeforeSolve";
    }
    case Result::IterativeSolveFail: {
      return "IterativeSolveFail";
    }
    default:
      assert(false && "unknown result");
      return "Unknown";
  }
}

class World::WorldImpl {
 private:
  // collision internal

  // solver internal
  bool need_warmup_ = true;
  int solver_state_num_;
  Eigen::VectorXf curr_position_;
  Eigen::VectorXf init_position_;
  Eigen::VectorXf velocity_;
  Eigen::SparseMatrix<float> mass_;
  Eigen::SparseMatrix<float> H_;  // iterative solver depends on this matrix !!
  Eigen::MatrixXf UHU_;
  Eigen::MatrixXf U_;
  Eigen::VectorXf HX_;
  std::vector<std::unique_ptr<SolverConstrain>> constrains_;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,
                           Eigen::Upper | Eigen::Lower,
                           Eigen::IncompleteCholesky<float>>
      iterative_solver_;

  // solver parameters (need recompute warmup)
  int low_freq_mode_num_;
  float dt_;

  Registry registry;

 public:
  Eigen::Vector3f constant_acce_field = {0.0f, 0.0f, -1.0f};
  int max_iteration = 5;
  int thread_num = 4;

  float get_dt() const { return dt_; }

  Result set_dt(float dt) {
    if (dt <= 0) {
      return Result::InvalidTimeStep;
    }
    dt_ = dt;
    need_warmup_ = true;
    return Result::Success;
  }

  int get_low_freq_mode_num() const { return low_freq_mode_num_; }

  Result set_low_freq_mode_num(int num) {
    if (num == 0) {
      return Result::InvalidLowFreqModeNum;
    }

    low_freq_mode_num_ = num;
    need_warmup_ = true;
    return Result::Success;
  }

  Result add_cloth(ClothConfig config, ClothHandle& handle) {
    if (!config.is_valid()) {
      return Result::InvalidConfig;
    }

    auto id = clothes_.add(Cloth{std::move(config)});
    if (id.is_empty()) {
      return Result::TooManyBody;
    }

    handle.value = id.get_raw();

    need_warmup_ = true;
    return Result::Success;
  }

  Result remove_cloth(const ClothHandle& handle) {
    Handle id{handle.value};
    if (!clothes_.remove(id)) {
      return Result::InvalidHandle;
    }

    need_warmup_ = true;
    return Result::Success;
  }

  Result update_cloth_config(const ClothHandle& handle, ClothConfig config) {
    Handle id{handle.value};
    Cloth* cloth = clothes_.get(id);
    if (!cloth) {
      return Result::InvalidHandle;
    }
    if (!config.is_valid()) {
      return Result::InvalidConfig;
    }
    *cloth = Cloth{std::move(config)};

    need_warmup_ = true;
    return Result::Success;
  }

  Result update_cloth_pinned_verts(const ClothHandle& handle,
                                   Eigen::VectorXf position) {
    const Cloth* c = clothes_.get(Handle{handle.value});
    if (!c) {
      return Result::InvalidHandle;
    }

    int pinned_num = c->get_pinned_vert_num();
    if (position.size() != 3 * pinned_num) {
      return Result::IncorrectPositionConstrainLength;
    }

    c->update_pinned_verts(curr_position_);
    return Result::Success;
  }

  Result get_cloth_position(const ClothHandle& handle,
                            Eigen::VectorXf& position) const {
    const Cloth* c = clothes_.get(Handle{handle.value});
    if (!c) {
      return Result::InvalidHandle;
    }

    Eigen::VectorXf out = c->get_position(curr_position_);
    if (out.size() != position.size()) {
      return Result::IncorrentOutputPositionLength;
    }

    position = out;
    return Result::Success;
  }

  void solver_reset() {
    need_warmup_ = true;
    curr_position_ = {};
    init_position_ = {};
    velocity_ = {};
    mass_ = {};
    H_ = {};
    UHU_ = {};
    U_ = {};
    HX_ = {};
    constrains_.clear();
    solver_state_num_ = 0;
  }

  Result solver_init() {
    solver_state_num_ = 0;
    for (auto& c : clothes_.get_dense_data()) {
      solver_state_num_ += c.solver_state_num();
    }

    int current_state_offset = 0;
    curr_position_.resize(solver_state_num_);
    init_position_.resize(solver_state_num_);
    velocity_.resize(solver_state_num_);
    std::vector<Eigen::Triplet<float>> mass_triplets;
    std::vector<Eigen::Triplet<float>> AA_triplets;

    for (auto& c : clothes_.get_dense_data()) {
      auto init_data = c.init_solver(current_state_offset);
      mass_triplets.insert(mass_triplets.end(), init_data.mass.begin(),
                           init_data.mass.end());
      AA_triplets.insert(AA_triplets.end(), init_data.weighted_AA.begin(),
                         init_data.weighted_AA.end());
      constrains_.insert(constrains_.end(), init_data.constrains.begin(),
                         init_data.constrains.end());
      init_position_(Eigen::seqN(current_state_offset, c.solver_state_num())) =
          init_data.init_solver_state;

      current_state_offset += c.solver_state_num();
    }

    curr_position_ = init_position_;
    for (int i = 0; i < solver_state_num_; ++i) {
      velocity_(i) = 0;
    }

    // set other solver matrices
    int num = 3 * solver_state_num_;
    mass_.resize(num, num);
    mass_.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
    Eigen::SparseMatrix<float> AA(num, num);
    AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());
    H_ = (mass_ / (dt_ * dt_) + AA);
    Eigen::ArpackGeneralizedSelfAdjointEigenSolver<
        Eigen::SparseMatrix<float>,
        Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>>
        eigen_solver;
    low_freq_mode_num_ = std::min(low_freq_mode_num_, num);
    eigen_solver.compute(H_, low_freq_mode_num_, "SM");
    if (eigen_solver.info() != Eigen::Success) {
      // SPDLOG_ERROR("eigen decomposition fail");
      // TODO: better signal warning
      return Result::EigenDecompositionfail;
    }
    U_ = eigen_solver.eigenvectors();
    UHU_ = eigen_solver.eigenvalues();
    HX_ = H_ * init_position_;

    // conjugate gradient solver depends on H_ !!
    iterative_solver_.setMaxIterations(max_iteration);
    iterative_solver_.compute(H_);
    if (iterative_solver_.info() != Eigen::Success) {
      // SPDLOG_ERROR("iterative solver decomposition fail");
      // TODO: signal warning
      return Result::IterativeSolverInitFail;
    }

    need_warmup_ = false;
    return Result::Success;
  }

  Result step() {
    if (need_warmup_) {
      return Result::NeedInitSolverBeforeSolve;
    }

    // basic linear velocity term
    Eigen::VectorXf rhs =
        (mass_ / dt_ / dt_) * curr_position_ + (mass_ / dt_) * velocity_ +
        mass_ * constant_acce_field.replicate(solver_state_num_, 1);

    // thread local buffer for rhs
    std::vector<Eigen::VectorXf> buffers(
        thread_num, Eigen::VectorXf::Zero(3 * solver_state_num_));
    // project constrains
#pragma omp parallel for num_threads(thread_num)
    for (const auto& c : constrains_) {
      Eigen::VectorXf& buffer = buffers[omp_get_thread_num()];
      c->project(curr_position_, buffer);
    }
    // merge thread local rhs back to global rhs
    for (Eigen::VectorXf& buffer : buffers) {
      rhs += buffer;
    }

    // sub space solve
    Eigen::VectorXf b = U_.transpose() * (rhs - HX_);
    Eigen::VectorXf q = b.array() / UHU_.array();
    Eigen::VectorXf subspace_sol = init_position_ + U_ * q;

    // iterative global solve
    Eigen::VectorXf sol = iterative_solver_.solveWithGuess(rhs, subspace_sol);
    if (iterative_solver_.info() != Eigen::Success &&
        iterative_solver_.info() != Eigen::NoConvergence) {
      return Result::IterativeSolveFail;
    }

    // CCD

    velocity_ = (sol - curr_position_) / dt_;
    curr_position_ = sol;

    return Result::Success;
  }
};

World::World() { impl_ = std::make_unique<silk::World::WorldImpl>(); }

World::~World() = default;

World::World(World&&) = default;

World& World::operator=(World&&) = default;

Result World::add_cloth(ClothConfig config, ClothHandle& handle) {
  return impl_->add_cloth(std::move(config), handle);
}

Result World::remove_cloth(const ClothHandle& handle) {
  return impl_->remove_cloth(handle);
}

Result World::update_cloth_config(const ClothHandle& handle,
                                  ClothConfig config) {
  return impl_->update_cloth_config(handle, std::move(config));
}

Result World::update_cloth_pinned_verts(const ClothHandle& handle,
                                        Eigen::VectorXf position) {
  return impl_->update_cloth_pinned_verts(handle, position);
}

Result World::get_cloth_position(const ClothHandle& handle,
                                 Eigen::VectorXf& position) const {
  return impl_->get_cloth_position(handle, position);
}

Eigen::Vector3f World::get_constant_acce_field() const {
  return impl_->constant_acce_field;
}

void World::set_constant_acce_field(Eigen::Vector3f acce) {
  impl_->constant_acce_field = std::move(acce);
}

// TODO: impl int range check
int World::get_max_iteration() const { return impl_->max_iteration; }

void World::set_max_iterations(int iter) { impl_->max_iteration = iter; }

int World::get_thread_num() const { return impl_->thread_num; }

void World::set_thread_num(int num) { impl_->thread_num = num; }

float World::get_dt() const { return impl_->get_dt(); }

Result World::set_dt(float dt) { return impl_->set_dt(dt); }

int World::get_low_freq_mode_num() const {
  return impl_->get_low_freq_mode_num();
}

Result World::set_low_freq_mode_num(int num) {
  return impl_->set_low_freq_mode_num(num);
}

void World::solver_reset() { impl_->solver_reset(); }

Result World::solver_init() { return impl_->solver_init(); }

Result World::step() { return impl_->step(); }

}  // namespace silk
