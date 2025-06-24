#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "cloth.hpp"
#include "common_types.hpp"
#include "physical_body.hpp"
#include "resource_manager.hpp"

class RigidBody : public PhysicalBody {
 public:
  SolverInitData compute_solver_init_data() const override;
};

struct PhysicalBodyPositionOffset {
  PhysicalBody* body;
  uint32_t offset;
};

class Collider;
class PositionConstrain;

enum class WorldResult : int {
  Success,
  EigenDecompositionfail,
  IterativeSolverInitFail
};

class World {
 private:
  // solver internal
  Eigen::VectorXf position_;         // position
  Eigen::VectorXf velocity_;         // velocity
  Eigen::SparseMatrix<float> mass_;  // mass
  Eigen::MatrixXf UHU_;
  Eigen::MatrixXf U_;
  std::vector<PhysicalBodyPositionOffset> body_position_offsets_;
  uint32_t total_vert_num_;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,
                           Eigen::Upper | Eigen::Lower,
                           Eigen::IncompleteCholesky<float>>
      iterative_solver_;

  // solver parameters
  uint32_t low_freq_mode_num_;
  float dt_;

  // entities
  ResourceManager<Cloth> clothes_;

  World() = default;

 public:
  Eigen::Vector3f constant_acce_field_ = {0.0f, 0.0f, -1.0f};
  int max_iterations = 5;

  void solver_reset() {
    position_ = {};
    velocity_ = {};
    mass_ = {};
    UHU_ = {};
    U_ = {};
    body_position_offsets_ = {};
    total_vert_num_ = 0;
  }

  [[nodiscard]] WorldResult solver_init() {
    uint32_t body_num = clothes_.size();
    body_position_offsets_.resize(body_num);
    total_vert_num_ = 0;

    std::vector<Eigen::Triplet<float>> mass_triplets;
    std::vector<Eigen::Triplet<float>> AA_triplets;

    auto init_solver_for_body = [this, &mass_triplets,
                                 &AA_triplets](PhysicalBody& body) {
      // collect init data
      SolverInitData init_data = body.compute_solver_init_data();
      uint32_t offset = 3 * total_vert_num_;
      for (auto& t : init_data.mass) {
        mass_triplets.emplace_back(t.row() + offset, t.col() + offset,
                                   t.value());
      }
      for (auto& t : init_data.weighted_AA) {
        AA_triplets.emplace_back(t.row() + offset, t.col() + offset, t.value());
      }
      // for (auto& c : init_data.constrains) {
      //   s.constrains.emplace_back(std::move(c));
      // }

      // update offset
      body_position_offsets_.emplace_back(body, 3 * total_vert_num_);
      body.set_position_offset(3 * total_vert_num_);
      total_vert_num_ += body.get_vert_num();
    };

    for (Cloth& body : clothes_.get_dense_data()) {
      init_solver_for_body(dynamic_cast<PhysicalBody&>(body));
    }

    // set initial velocity and position
    velocity_ = Eigen::VectorXf::Zero(3 * total_vert_num_);
    position_.resize(3 * total_vert_num_);
    for (PhysicalBodyPositionOffset& o : body_position_offsets_) {
      uint32_t len = 3 * o.body->get_vert_num();
      position_(Eigen::seqN(o.offset, len)) = o.body->get_init_position();
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
      spdlog::error("eigen decomposition fail");
      return WorldResult::EigenDecompositionfail;
    }
    U_ = eigen_solver.eigenvectors();
    UHU_ = eigen_solver.eigenvalues();

    iterative_solver_.compute(H);
    if (iterative_solver_.info() != Eigen::Success) {
      spdlog::error("iterative solver decomposition fail");
      return WorldResult::IterativeSolverInitFail;
    }

    return WorldResult::Success;
  }
};
