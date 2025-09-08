#pragma once

#include <Eigen/Core>
#include <vector>

#include "collision.hpp"
#include "object_collider.hpp"

namespace silk {

class CollisionPipeline {
 public:
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  int partial_ccd_max_iter = 32;

  float min_toi = 0.05f;

  float collision_stiffness_base = 1e4f;
  float collision_stiffness_max = 1e4f;
  float collision_stiffness_growth = 16.0f;

  std::vector<Collision> find_collision(
      std::vector<ObjectCollider>& object_colliders, const Bbox& scene_bbox,
      float dt);

  void update_collision(const Eigen::VectorXf& solver_state_t0,
                        const Eigen::VectorXf& solver_state_t1,
                        std::vector<Collision>& collisions) const;

 private:
  Eigen::Array3f scene_ee_err_;
  Eigen::Array3f scene_vf_err_;
};

}  // namespace silk
