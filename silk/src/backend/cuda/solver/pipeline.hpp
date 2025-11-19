#pragma once

#include <Eigen/Core>
#include <vector>

#include "backend/cuda/collision/bbox.hpp"
#include "backend/cuda/collision/collision.hpp"
#include "backend/cuda/collision/pipeline.hpp"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/object_state.hpp"
#include "backend/cuda/solver/barrier_constrain.hpp"

namespace silk::cuda {

class SolverPipeline {
 public:
  Eigen::Vector3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;
  float eps = 1e-6f;

 private:
  CollisionPipeline collision_pipeline;

 public:
  void clear(Registry& registry);
  void reset(Registry& registry);
  bool step(Registry& registry);

 private:
  bool init(Registry& registry, ObjectState& global_state);
  Bbox compute_scene_bbox(Registry& registry);
  void compute_barrier_constrain(const Eigen::VectorXf& state,
                                 const std::vector<Collision>& collisions,
                                 BarrierConstrain& barrier);
};

}  // namespace silk::cuda
