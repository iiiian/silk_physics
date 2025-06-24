#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cstdint>
#include <memory>
#include <vector>

#include "common_types.hpp"

class SolverConstrain {
 public:
  virtual ~SolverConstrain() = default;

  virtual void project(uint32_t vert_offset, const Eigen::VectorXf& verts,
                       Eigen::VectorXf& out) const = 0;
};

struct SolverInitData {
  std::vector<Eigen::Triplet<float>> mass;
  std::vector<Eigen::Triplet<float>> weighted_AA;
  std::vector<std::unique_ptr<SolverConstrain>> constrains;
};

class PhysicalBody {
 public:
  virtual ~PhysicalBody() = default;

  virtual SolverInitData compute_solver_init_data() const = 0;
  virtual uint32_t get_vert_num() const = 0;
  virtual const RMatrixX3f& get_init_position() const = 0;
  virtual uint32_t get_position_offset() const = 0;
  virtual void set_position_offset(uint32_t offset) = 0;
};
