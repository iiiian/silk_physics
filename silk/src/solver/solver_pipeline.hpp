#pragma once

#include <Eigen/Core>
#include <vector>

#include "../barrier_constrain.hpp"
#include "../bbox.hpp"
#include "../collision.hpp"
#include "../collision_pipeline.hpp"
#include "../ecs.hpp"

namespace silk {

class SolverPipeline {
 public:
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;
  float eps = 1e-6f;

 private:
  // we do cache collision to next step
  std::vector<Collision> collisions_;

 public:
  void clear(Registry& registry);
  void reset(Registry& registry);
  bool init(Registry& registry);
  bool step(Registry& registry, CollisionPipeline& collision_pipeline);

 private:
  void gather_solver_state(Registry& registry, Eigen::VectorXf& state,
                           Eigen::VectorXf& state_velocity);
  Bbox compute_scene_bbox(const Eigen::VectorXf& state);
  void cleanup_collisions(Registry& registry);
  Eigen::VectorXf gather_pin_constrain(Registry& registry, int state_num);
  BarrierConstrain compute_barrier_constrain(const Eigen::VectorXf& state);

  void enforce_barrier_constrain(const BarrierConstrain& barrier_constrain,
                                 const Bbox& scene_bbox,
                                 Eigen::VectorXf& state);
};

}  // namespace silk
