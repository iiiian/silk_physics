#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <memory>
#include <optional>
#include <unsupported/Eigen/ArpackSupport>
#include <vector>

#include "common_types.hpp"

namespace eg = Eigen;

class SolverConstrain {
 public:
  virtual void project(uint32_t vert_offset, const eg::VectorXf& verts,
                       eg::VectorXf& out) const = 0;
};

struct SolverInitData {
  std::vector<eg::Triplet<float>> mass;
  std::vector<eg::Triplet<float>> weighted_AA;
  std::vector<std::unique_ptr<SolverConstrain>> constrains;
};

class PhysicalObject {
 public:
  virtual uint32_t get_vert_num() const = 0;
  virtual const eg::VectorXf& get_verts() const = 0;
  virtual void set_verts(const eg::VectorXf& verts) = 0;
  virtual SolverInitData compute_solver_init_data() const = 0;
};

class World {
 private:
  // std::vector<std::unique_ptr<PDConstrain>> constrains;
  std::vector<PhysicalObject*> physical_objs_;
  std::vector<uint32_t> physical_obj_vert_offsets_;
  eg::VectorXf position_;         // position
  eg::VectorXf velocity_;         // velocity
  eg::SparseMatrix<float> mass_;  // mass
  eg::MatrixXf UHU_;
  eg::MatrixXf U_;
  uint32_t total_vert_num_;
  uint32_t low_freq_mode_num_;
  float dt_;

  // Eigen::ConjugateGradient<eg::SparseMatrix<float>, eg::Upper | eg::Lower,
  //                          eg::IncompleteCholesky<float>>
  //     iterative_solver_;

  World() = default;

 public:
  eg::Vector3f constant_acce_field_ = {0.0f, 0.0f, -1.0f};
  int max_iterations = 5;

  // NDBPDSolver(NDBPDSolver&) = delete;
  // NDBPDSolver(NDBPDSolver&&) = default;
  // NDBPDSolver& operator=(NDBPDSolver&) = delete;
  // NDBPDSolver& operator=(NDBPDSolver&&) = default;

  void solver_reset() {}

  bool solver_init() {
    uint32_t obj_num = physical_objs_.size();
    physical_obj_vert_offsets_.resize(obj_num);
    total_vert_num_ = 0;

    std::vector<eg::Triplet<float>> mass_triplets;
    std::vector<eg::Triplet<float>> AA_triplets;

    for (uint32_t i = 0; i < obj_num; ++i) {
      PhysicalObject* obj = physical_objs[i];
      // collect init data
      SolverInitData init_data = obj->compute_solver_init_data();
      uint32_t offset = 3 * s.total_vert_num_;
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
      s.physical_obj_vert_offsets_[i] = offset;
      s.total_vert_num_ += obj->get_vert_num();
    }

    // set initial velocity and position
    s.velocity_ = eg::VectorXf::Zero(3 * s.total_vert_num_);
    s.position_.resize(3 * s.total_vert_num_);
    for (uint32_t i = 0; i < obj_num; ++i) {
      PhysicalObject* obj = s.physical_objs_[i];
      uint32_t offset = s.physical_obj_vert_offsets_[i];
      uint32_t size = 3 * obj->get_vert_num();
      s.position_(eg::seqN(offset, size)) = obj->get_verts();
    }

    // set other solver matrices
    s.mass_.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
    eg::SparseMatrix<float> AA;
    AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());

    eg::SparseMatrix<float> H = (s.mass_ / (dt * dt) + AA);
    eg::ArpackGeneralizedSelfAdjointEigenSolver<
        eg::SparseMatrix<float>, eg::SimplicialLLT<eg::SparseMatrix<float>>,
        true>
        eigen_solver;
    eigen_solver.compute(H, low_freq_mode_num, "SM");
    if (eigen_solver.info() != eg::Success) {
      spdlog::error("eigen decomposition fail");
      return std::nullopt;
    }

    s.U_ = eigen_solver.eigenvectors();
    s.UHU_ = eigen_solver.eigenvalues();

    // s.iterative_solver_.compute(H);
    // if (s.iterative_solver_.info() != eg::Success) {
    //   spdlog::error("iterative solver decomposition fail");
    //   return std::nullopt;
    // }

    return s;
  }
};
