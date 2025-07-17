#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>  // fix a bug in eigen arpack that miss this include
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "cloth.hpp"
#include "physical_body.hpp"
#include "resource_manager.hpp"
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

struct BodyPositionOffset {
  PhysicalBody* body;
  int offset;
};

struct FaceCollider {
  PhysicalBody* body;
  int v1;
  int v2;
  int v3;
};

class World::WorldImpl {
 private:
  // collision internal

  // solver internal
  bool need_warmup_ = true;
  int total_vert_num_;
  Eigen::VectorXf curr_position_;
  Eigen::VectorXf init_position_;
  Eigen::VectorXf velocity_;
  Eigen::SparseMatrix<float> mass_;
  Eigen::SparseMatrix<float> H_;  // iterative solver depends on this matrix !!
  Eigen::MatrixXf UHU_;
  Eigen::MatrixXf U_;
  Eigen::VectorXf HX_;
  std::vector<BodyPositionOffset> body_position_offsets_;
  std::vector<std::unique_ptr<SolverConstrain>> constrains_;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,
                           Eigen::Upper | Eigen::Lower,
                           Eigen::IncompleteCholesky<float>>
      iterative_solver_;

  // solver parameters (need recompute warmup)
  int low_freq_mode_num_;
  float dt_;

  // entities
  ResourceManager<Cloth> clothes_;

  void init_solver_body_offset() {
    body_position_offsets_.clear();
    total_vert_num_ = 0;

    auto init_offset = [this](PhysicalBody& body) {
      body_position_offsets_.emplace_back(
          BodyPositionOffset{&body, 3 * total_vert_num_});
      body.set_position_offset(3 * total_vert_num_);
      total_vert_num_ += body.get_vert_num();
    };

    for (Cloth& body : clothes_.get_dense_data()) {
      init_offset(body);
    }
  }

  void init_solver_position_and_velocity() {
    velocity_ = Eigen::VectorXf::Zero(3 * total_vert_num_);
    curr_position_.resize(3 * total_vert_num_);
    for (BodyPositionOffset& body_offset : body_position_offsets_) {
      int num = 3 * body_offset.body->get_vert_num();
      curr_position_(Eigen::seqN(body_offset.offset, num)) =
          body_offset.body->get_init_position();
    }
    init_position_ = curr_position_;
  }

  PhysicalBody* resolve_handle(const Handle& handle) {
    ResourceHandle r_handle{handle.value};
    switch (handle.type) {
      case HandleType::Cloth: {
        return dynamic_cast<PhysicalBody*>(clothes_.get_resources(r_handle));
      }
      case HandleType::RigidBody:
        assert(false && "not impl");
        return nullptr;
      case HandleType::SoftBody:
        assert(false && "not impl");
        return nullptr;
      case HandleType::Hair:
        assert(false && "not impl");
        return nullptr;
      case HandleType::Collider:
        assert(false && "not impl");
        return nullptr;
      default:
        assert(false && "unknown handle type");
        return nullptr;
    }
  }

  const PhysicalBody* resolve_handle(const Handle& handle) const {
    ResourceHandle r_handle{handle.value};
    switch (handle.type) {
      case HandleType::Cloth: {
        return dynamic_cast<const PhysicalBody*>(
            clothes_.get_resources(r_handle));
      }
      case HandleType::RigidBody:
        assert(false && "not impl");
        return nullptr;
      case HandleType::SoftBody:
        assert(false && "not impl");
        return nullptr;
      case HandleType::Hair:
        assert(false && "not impl");
        return nullptr;
      case HandleType::Collider:
        assert(false && "not impl");
        return nullptr;
      default:
        assert(false && "unknown handle type");
        return nullptr;
    }
  }

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

  Result add_cloth(ClothConfig config, Handle& handle) {
    if (!config.is_valid()) {
      return Result::InvalidConfig;
    }

    auto resource_handle = clothes_.add_resource(Cloth{std::move(config)});
    if (!resource_handle) {
      return Result::TooManyBody;
    }

    handle.type = HandleType::Cloth;
    handle.value = resource_handle->get_value();

    need_warmup_ = true;
    return Result::Success;
  }

  Result remove_cloth(const Handle& handle) {
    if (handle.type != HandleType::Cloth) {
      return Result::InvalidHandle;
    }
    ResourceHandle r_handle{handle.value};
    if (!clothes_.remove_resource(r_handle)) {
      return Result::InvalidHandle;
    }

    need_warmup_ = true;
    return Result::Success;
  }

  Result update_cloth(ClothConfig config, const Handle& handle) {
    if (handle.type != HandleType::Cloth) {
      return Result::InvalidHandle;
    }
    ResourceHandle resource_handle{handle.value};
    Cloth* cloth = clothes_.get_resources(resource_handle);
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
    body_position_offsets_.clear();
    constrains_.clear();
    total_vert_num_ = 0;
  }

  Result solver_init() {
    init_solver_body_offset();
    init_solver_position_and_velocity();
    std::vector<Eigen::Triplet<float>> mass_triplets;
    std::vector<Eigen::Triplet<float>> AA_triplets;

    auto collect_solver_init_data = [this, &mass_triplets,
                                     &AA_triplets](PhysicalBody& body) {
      SolverInitData init_data = body.compute_solver_init_data();
      for (auto& t : init_data.mass) {
        mass_triplets.emplace_back(std::move(t));
      }
      for (auto& t : init_data.weighted_AA) {
        AA_triplets.emplace_back(std::move(t));
      }
      for (auto& t : init_data.constrains) {
        constrains_.emplace_back(std::move(t));
      }
    };

    for (Cloth& body : clothes_.get_dense_data()) {
      collect_solver_init_data(dynamic_cast<PhysicalBody&>(body));
    }

    // set other solver matrices
    int num = 3 * total_vert_num_;
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
        mass_ * constant_acce_field.replicate(total_vert_num_, 1);

    // thread local buffer for rhs
    std::vector<Eigen::VectorXf> buffers(
        thread_num, Eigen::VectorXf::Zero(3 * total_vert_num_));
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

  Result update_position_constrain(
      const Handle& handle, Eigen::Ref<const Eigen::VectorXf> position) {
    PhysicalBody* body = resolve_handle(handle);
    if (!body) {
      return Result::InvalidHandle;
    }

    auto pinned_verts = body->get_pinned_verts();
    if (position.size() != 3 * pinned_verts.size()) {
      return Result::IncorrectPositionConstrainLength;
    }
    int offset = body->get_position_offset();
    for (int idx = 0; idx < pinned_verts.size(); ++idx) {
      curr_position_(Eigen::seqN(offset + 3 * pinned_verts(idx), 3)) =
          position(Eigen::seqN(3 * idx, 3));
    }

    return Result::Success;
  }

  Result get_current_position(const Handle& handle,
                              Eigen::Ref<Eigen::VectorXf> position) const {
    const PhysicalBody* body = resolve_handle(handle);
    if (!body) {
      return Result::InvalidHandle;
    }

    int offset = body->get_position_offset();
    int num = 3 * body->get_vert_num();
    if (position.size() != num) {
      return Result::IncorrentOutputPositionLength;
    }

    position = curr_position_(Eigen::seqN(offset, num));
    return Result::Success;
  }
};

World::World() { impl_ = std::make_unique<silk::World::WorldImpl>(); }

World::~World() = default;

World::World(World&&) = default;

World& World::operator=(World&&) = default;

[[nodiscard]] Result World::add_cloth(ClothConfig config, Handle& handle) {
  return impl_->add_cloth(std::move(config), handle);
}

[[nodiscard]] Result World::remove_cloth(const Handle& handle) {
  return impl_->remove_cloth(handle);
}

[[nodiscard]] Result World::update_cloth(ClothConfig config,
                                         const Handle& handle) {
  return impl_->update_cloth(std::move(config), handle);
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

[[nodiscard]] Result World::set_dt(float dt) { return impl_->set_dt(dt); }

int World::get_low_freq_mode_num() const {
  return impl_->get_low_freq_mode_num();
}

[[nodiscard]] Result World::set_low_freq_mode_num(int num) {
  return impl_->set_low_freq_mode_num(num);
}

void World::solver_reset() { impl_->solver_reset(); }

[[nodiscard]] Result World::solver_init() { return impl_->solver_init(); }

[[nodiscard]] Result World::step() { return impl_->step(); }

[[nodiscard]] Result World::update_position_constrain(
    const Handle& handle, Eigen::Ref<const Eigen::VectorXf> position) {
  return impl_->update_position_constrain(handle, position);
}

[[nodiscard]] Result World::get_current_position(
    const Handle& handle, Eigen::Ref<Eigen::VectorXf> position) const {
  return impl_->get_current_position(handle, position);
}

}  // namespace silk
