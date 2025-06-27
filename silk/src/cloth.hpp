#pragma once

#include <optional>
#include <silk/silk.hpp>

#include "physical_body.hpp"
#include "solver.hpp"

namespace silk {

using Matrix69f = Eigen::Matrix<float, 6, 9>;

class ClothElasticConstrain : public SolverConstrain {
  Matrix69f jacobian_op_;
  Eigen::Vector3i vidx_;
  uint32_t offset_;
  float weight_;

 public:
  ClothElasticConstrain(Matrix69f jacobian_op, Eigen::Vector3i vert_indexes,
                        uint32_t offset, float weight);

  // impl solver constrain interface
  void project(const Eigen::VectorXf& verts,
               Eigen::VectorXf& out) const override;
};

class Cloth : public PhysicalBody {
  ClothConfig cfg_;
  uint32_t solver_position_offset_ = 0;  // managed by solver

  // compute vectorized jacobian operator given initial triangle vertex position
  // v1, v2, v3.
  // return nullopt if triangle is degenerated
  std::optional<Matrix69f> vectorized_jacobian_operator(
      Eigen::Ref<const Eigen::Vector3f> v1,
      Eigen::Ref<const Eigen::Vector3f> v2,
      Eigen::Ref<const Eigen::Vector3f> v3, float zero_threshold = 1e-8) const;

 public:
  Cloth(ClothConfig config);

  // impl physical body interface
  uint32_t get_vert_num() const override;
  Eigen::Ref<const Eigen::VectorXf> get_init_position() const override;
  uint32_t get_position_offset() const override;
  void set_position_offset(uint32_t offset) override;
  Eigen::Ref<const Eigen::VectorXi> get_pinned_verts() const override;

  SolverInitData compute_solver_init_data() const override;
};

}  // namespace silk
