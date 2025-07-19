#pragma once

#include <optional>
#include <silk/silk.hpp>

#include "collision/collision.hpp"
#include "physical_body.hpp"
#include "solver.hpp"

namespace silk {

using Matrix69f = Eigen::Matrix<float, 6, 9>;

class ClothElasticConstrain : public SolverConstrain {
  Matrix69f jacobian_op_;
  Eigen::Vector3i vert_indexes_;
  int offset_;
  float weight_;

 public:
  ClothElasticConstrain(Matrix69f jacobian_op, Eigen::Vector3i vert_indexes,
                        int offset, float weight);

  // impl solver constrain interface
  void project(const Eigen::VectorXf& verts,
               Eigen::VectorXf& out) const override;
};

class SolverCloth : public ISolverBody, public IObstacle {
 private:
  ClothConfig cfg_;
  int solver_position_offset_ = 0;  // managed by solver

 public:
  SolverCloth(ClothConfig config);

  // impl solver body interface
  int get_vert_num() const override;
  Eigen::Ref<const Eigen::VectorXf> get_init_position() const override;
  int get_position_offset() const override;
  void set_position_offset(int offset) override;
  Eigen::Ref<const Eigen::VectorXi> get_pinned_verts() const override;
  SolverInitData compute_solver_init_data() const override;

  // impl obstacle interface
  std::vector<MeshCollider> init_mesh_colliders() const override;
  void update_mesh_colliders(
      std::vector<MeshCollider>& mesh_colliders) const override;
  MeshColliderDetail get_mesh_collider_detail(int index) const override;
  void resolve_impact(MeshColliderImpact impact) override;
  ObstacleStatus get_obstacle_status() const override;

 private:
  // compute vectorized jacobian operator given initial triangle vertex position
  // v1, v2, v3.
  // return nullopt if triangle is degenerated
  std::optional<Matrix69f> vectorized_jacobian_operator(
      Eigen::Ref<const Eigen::Vector3f> v1,
      Eigen::Ref<const Eigen::Vector3f> v2,
      Eigen::Ref<const Eigen::Vector3f> v3, float zero_threshold = 1e-8) const;
};

}  // namespace silk
