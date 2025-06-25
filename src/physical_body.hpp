#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "solver.hpp"

class PhysicalBody {
 public:
  virtual ~PhysicalBody() = default;

  virtual uint32_t get_vert_num() const = 0;
  virtual Eigen::Ref<const Eigen::VectorXf> get_init_position() const = 0;
  virtual uint32_t get_position_offset() const = 0;
  virtual void set_position_offset(uint32_t offset) = 0;
  virtual Eigen::Ref<const Eigen::VectorXi> get_pinned_verts() const = 0;

  virtual SolverInitData compute_solver_init_data() const = 0;
};
