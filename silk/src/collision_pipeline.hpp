#pragma once

#include <Eigen/Core>
#include <vector>

#include "collision.hpp"
#include "object_collider.hpp"

namespace silk {

class CollisionPipeline {
 public:
  // deprecated
  float toi_tolerance = 0.05;
  int toi_bisect_it = 4;
  float eps = 1e-6;

  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  int partial_ccd_max_iter = 32;

  float collision_stiffness_base = 1e8f;
  float collision_stiffness_max = 1e8f;
  float collision_stiffness_growth = 16.0f;

  std::vector<Collision> find_collision(
      std::vector<ObjectCollider>& object_colliders, float dt);

  void update_collision(const Eigen::VectorXf& solver_state_t0,
                        const Eigen::VectorXf& solver_state_t1,
                        std::vector<Collision>& collisions);

 private:
  Eigen::Array3f scene_ee_err_;
  Eigen::Array3f scene_vf_err_;
};

}  // namespace silk
