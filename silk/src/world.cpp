
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>  // fix a bug in eigen arpack that miss this include
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "cloth.hpp"
#include "common_types.hpp"
#include "physical_body.hpp"
#include "resource_manager.hpp"
#include "solver.hpp"

struct PhysicalBodyPositionOffset {
  PhysicalBody* body;
  uint32_t offset;
};

class World::WorldImpl {
 private:
  // solver internal
  bool need_warmup = true;
  uint32_t total_vert_num_;
  Eigen::VectorXf position_;
  Eigen::VectorXf init_position_;
  Eigen::VectorXf velocity_;
  Eigen::SparseMatrix<float> mass_;
  Eigen::MatrixXf UHU_;
  Eigen::MatrixXf U_;
  Eigen::VectorXf HX_;
  std::vector<PhysicalBodyPositionOffset> body_position_offsets_;
  std::vector<std::unique_ptr<SolverConstrain>> constrains_;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,
                           Eigen::Upper | Eigen::Lower,
                           Eigen::IncompleteCholesky<float>>
      iterative_solver_;

  // solver parameters (need recompute warmup)
  uint32_t low_freq_mode_num_;
  float dt_;

  // entities
  ResourceManager<Cloth> clothes_;

  WorldImpl() = default;

  void init_solver_body_offset() {
    uint32_t body_num = clothes_.size();
    body_position_offsets_.resize(body_num);
    total_vert_num_ = 0;

    auto init_offset = [this](PhysicalBody& body) {
      body_position_offsets_.emplace_back(
          PhysicalBodyPositionOffset{&body, 3 * total_vert_num_});
      body.set_position_offset(3 * total_vert_num_);
      total_vert_num_ += body.get_vert_num();
    };

    for (Cloth& body : clothes_.get_dense_data()) {
      init_offset(body);
    }
  }

  void init_solver_position_and_velocity() {
    velocity_ = Eigen::VectorXf::Zero(3 * total_vert_num_);
    position_.resize(3 * total_vert_num_);
    for (auto& offset : body_position_offsets_) {
      uint32_t len = 3 * offset.body->get_vert_num();
      position_(Eigen::seqN(offset.offset, len)) =
          offset.body->get_init_position();
    }
    init_position_ = position_;
  }

  PhysicalBody* resolve_handle(const Handle& handle) {
    ResourceHandle r_handle{handle.value};
    switch (handle.type) {
      case HandleType::Cloth: {
        return dynamic_cast<PhysicalBody*>(clothes_.get_resources(r_handle));
      }
      case HandleType::RigidBody:
        assert(false && "not impl");
        break;
      case HandleType::SoftBody:
        assert(false && "not impl");
        break;
      case HandleType::Hair:
        assert(false && "not impl");
        break;
      case HandleType::Collider:
        assert(false && "not impl");
        break;
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
        break;
      case HandleType::SoftBody:
        assert(false && "not impl");
        break;
      case HandleType::Hair:
        assert(false && "not impl");
        break;
      case HandleType::Collider:
        assert(false && "not impl");
        break;
    }
  }

 public:
  Eigen::Vector3f constant_acce_field = {0.0f, 0.0f, -1.0f};
  uint32_t max_iterations = 5;
  uint32_t thread_num = 4;

  float get_dt() const { return dt_; }

  WorldResult set_dt(float dt) {
    if (dt <= 0) {
      return WorldResult::InvalidTimeStep;
    }
    dt_ = dt;
    need_warmup = true;
    return WorldResult::Success;
  }

  uint32_t get_low_freq_mode_num() const { return low_freq_mode_num_; }

  WorldResult get_low_freq_mode_num(uint32_t num) {
    if (num == 0) {
      return WorldResult::InvalidLowFreqModeNum;
    }

    low_freq_mode_num_ = num;
    need_warmup = true;
    return WorldResult::Success;
  }

  WorldResult add_cloth(ClothConfig config, Handle& handle) {
    if (!config.is_valid()) {
      return WorldResult::InvalidConfig;
    }

    Cloth cloth{config};
    auto r_handle = clothes_.add_resource(std::move(cloth));
    if (!r_handle) {
      return WorldResult::TooManyBody;
    }

    handle.type = HandleType::Cloth;
    handle.value = r_handle->get_value();

    need_warmup = true;
    return WorldResult::Success;
  }

  WorldResult remove_cloth(const Handle& handle) {
    if (handle.type != HandleType::Cloth) {
      return WorldResult::InvalidHandle;
    }
    ResourceHandle r_handle{handle.value};
    if (!clothes_.remove_resource(r_handle)) {
      return WorldResult::InvalidHandle;
    }

    need_warmup = true;
    return WorldResult::Success;
  }

  WorldResult update_cloth(ClothConfig config, const Handle& handle) {
    if (handle.type != HandleType::Cloth) {
      return WorldResult::InvalidHandle;
    }
    ResourceHandle r_handle{handle.value};
    Cloth* cloth = clothes_.get_resources(r_handle);
    if (!cloth) {
      return WorldResult::InvalidHandle;
    }
    if (!config.is_valid()) {
      return WorldResult::InvalidConfig;
    }
    *cloth = Cloth{config};

    need_warmup = true;
    return WorldResult::Success;
  }

  void solver_reset() {
    need_warmup = true;
    position_ = {};
    init_position_ = {};
    velocity_ = {};
    mass_ = {};
    UHU_ = {};
    U_ = {};
    HX_ = {};
    body_position_offsets_.clear();
    constrains_.clear();
    total_vert_num_ = 0;
  }

  WorldResult init_solver() {
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
    mass_.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
    Eigen::SparseMatrix<float> AA;
    AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());
    Eigen::SparseMatrix<float> H = (mass_ / (dt_ * dt_) + AA);
    Eigen::ArpackGeneralizedSelfAdjointEigenSolver<
        Eigen::SparseMatrix<float>,
        Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>, true>
        eigen_solver;
    eigen_solver.compute(H, low_freq_mode_num_, "SM");
    if (eigen_solver.info() != Eigen::Success) {
      // spdlog::error("eigen decomposition fail");
      // TODO: better signal warning
      return WorldResult::EigenDecompositionfail;
    }
    U_ = eigen_solver.eigenvectors();
    UHU_ = eigen_solver.eigenvalues();
    HX_ = H * init_position_;

    iterative_solver_.compute(H);
    if (iterative_solver_.info() != Eigen::Success) {
      // spdlog::error("iterative solver decomposition fail");
      // TODO: signal warning
      return WorldResult::IterativeSolverInitFail;
    }

    need_warmup = false;
    return WorldResult::Success;
  }

  WorldResult step() {
    if (need_warmup) {
      return WorldResult::NeedInitSolverBeforeSolve;
    }

    // basic linear velocity term
    Eigen::VectorXf rhs =
        (mass_ / dt_ / dt_) * velocity_ + (mass_ / dt_) * velocity_ +
        mass_ * constant_acce_field.replicate(total_vert_num_, 1);

    // thread local rhs
    std::vector<Eigen::VectorXf> thread_local_rhs(
        thread_num, Eigen::VectorXf::Zero(3 * total_vert_num_));
    // project constrains
#pragma omp parallel for num_threads(thread_num)
    for (const auto& c : constrains_) {
      auto& local_rhs = thread_local_rhs[omp_get_thread_num()];
      c->project(position_, local_rhs);
    }
    // merge thread local rhs back to global rhs
    for (auto& r : thread_local_rhs) {
      rhs += r;
    }

    // sub space solve
    Eigen::VectorXf b = U_.transpose() * (rhs - HX_);
    Eigen::VectorXf q = b.array() / UHU_.array();
    Eigen::VectorXf subspace_sol = init_position_ + U_ * q;

    // iterative global solve
    Eigen::VectorXf sol = iterative_solver_.solveWithGuess(rhs, subspace_sol);
    if (iterative_solver_.info() != Eigen::Success &&
        iterative_solver_.info() != Eigen::NoConvergence) {
      return WorldResult::IterativeSolveFail;
    }

    // spdlog::info("itertive solver iteration {}",
    //              iterative_solver_.iterations());

    // detect and resolve collision and update velocity
    // the collision part is definitely half baked and its pretty janky
    // if (enable_collision) {
    //   collision_detector.detect(pV_, &future_V_, pF_,
    //                             ClothSolver::rtc_collision_callback, this);
    // }

    velocity_ = (sol - position_) / dt_;
    position_ = sol;

    return WorldResult::Success;
  }

  WorldResult update_position_constrain(
      const Handle& handle, Eigen::Ref<const Eigen::VectorXf> positions) {
    PhysicalBody* body = resolve_handle(handle);
    if (!body) {
      return WorldResult::InvalidHandle;
    }

    auto pinned_verts = body->get_pinned_verts();
    if (position_.size() != pinned_verts.size()) {
      return WorldResult::IncorrectPositionConstrainLength;
    }
    uint32_t offset = body->get_position_offset();
    for (auto idx : pinned_verts) {
      position_(Eigen::seqN(offset + 3 * idx, 3)) =
          positions(Eigen::seqN(3 * idx, 3));
    }

    return WorldResult::Success;
  }

  WorldResult get_current_position(
      const Handle& handle, Eigen::Ref<Eigen::VectorXf> positions) const {
    const PhysicalBody* body = resolve_handle(handle);
    if (!body) {
      return WorldResult::InvalidHandle;
    }

    uint32_t offset = body->get_position_offset();
    uint32_t len = 3 * body->get_vert_num();
    if (positions.size() != len) {
      return WorldResult::IncorrentOutputPositionLength;
    }

    positions = position_(Eigen::seqN(offset, len));
    return WorldResult::Success;
  }
};

[[nodiscard]] WorldResult World::add_cloth(ClothConfig config, Handle& handle) {
  return impl_->add_cloth(config, handle);
}

[[nodiscard]] WorldResult World::remove_cloth(const Handle& handle) {
  return impl_->remove_cloth(handle);
}

[[nodiscard]] WorldResult World::update_cloth(ClothConfig config,
                                              const Handle& handle) {
  return impl_->update_cloth(config, handle);
}

Eigen::Vector3f& World::constant_acce_field() {
  return impl_->constant_acce_field;
}

Eigen::Vector3f World::constant_acce_field() const {
  return impl_->constant_acce_field;
}

uint32_t& World::max_iterations() { return impl_->max_iterations; }

uint32_t World::max_iterations() const { return impl_->max_iterations; }

uint32_t& World::thread_num() { return impl_->thread_num; }

uint32_t World::thread_num() const { return impl_->thread_num; }

float World::get_dt() const { return impl_->get_dt(); }

[[nodiscard]] WorldResult World::set_dt(float dt) { return impl_->set_dt(dt); }

uint32_t World::get_low_freq_mode_num() const {
  return impl_->get_low_freq_mode_num();
}

[[nodiscard]] WorldResult World::get_low_freq_mode_num(uint32_t num) {
  return impl_->get_low_freq_mode_num(num);
}

void World::solver_reset() { impl_->solver_reset(); }

[[nodiscard]] WorldResult World::init_solver() { return impl_->init_solver(); }

[[nodiscard]] WorldResult World::step() { return impl_->step(); }

[[nodiscard]] WorldResult World::update_position_constrain(
    const Handle& handle, Eigen::Ref<const Eigen::VectorXf> positions) {
  return impl_->update_position_constrain(handle, positions);
}

[[nodiscard]] WorldResult World::get_current_position(
    const Handle& handle, Eigen::Ref<Eigen::VectorXf> positions) const {
  return impl_->get_current_position(handle, positions);
}
