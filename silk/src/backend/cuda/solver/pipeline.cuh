#pragma once

#include <cuda/buffer>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/collision.hpp"
#include "backend/cuda/collision/pipeline.hpp"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/object_state.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/barrier_constrain.hpp"

namespace silk::cuda {

class SolverPipeline {
 public:
  Vec3 const_acceleration = {0.0f, 0.0f, -1.0f};
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
  void compute_barrier_constrain(const cu::buffer<float>& state,
                                 const std::vector<Collision>& collisions,
                                 BarrierConstrain& barrier);
};

}  // namespace silk::cuda
