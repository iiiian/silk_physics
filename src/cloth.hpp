#pragma once

#include <optional>

#include "api.hpp"
#include "common_types.hpp"
#include "mesh.hpp"
#include "physical_body.hpp"

class ClothSolverConstrain : public SolverConstrain {
  Matrix69f jacobian_op_;
  Eigen::Vector3i vidx_;
  float weight_;

 public:
  ClothSolverConstrain(Matrix69f jacobian_op, Eigen::Vector3i vert_indexes,
                       float weight);

  // impl solver constrain interface
  void project(uint32_t vert_offset, const Eigen::VectorXf& verts,
               Eigen::VectorXf& out) const override;
};

class Cloth : public PhysicalBody {
  Mesh mesh_;
  ClothConfig config_;
  uint32_t solver_position_offset_ = 0;  // managed by solver

  // compute vectorized jacobian operator given initial triangle vertex position
  // v1, v2, v3.
  // return nullopt if triangle is degenerated
  std::optional<Matrix69f> vectorized_jacobian_operator(
      Eigen::Ref<const Eigen::Vector3f> v1,
      Eigen::Ref<const Eigen::Vector3f> v2,
      Eigen::Ref<const Eigen::Vector3f> v3, float zero_threshold = 1e-8) const;

 public:
  Cloth(Mesh mesh, ClothConfig config);

  // impl physical body interface
  SolverInitData compute_solver_init_data() const override;
  uint32_t get_vert_num() const override;
  const RMatrixX3f& get_init_position() const override;
  uint32_t get_position_offset() const override;
  void set_position_offset(uint32_t offset) override;
};
