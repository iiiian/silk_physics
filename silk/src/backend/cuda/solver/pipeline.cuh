#pragma once

#include <cuda/buffer>
#include <optional>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

enum class ADMMError { NothingToSolve };

class ADMMSolver {
 public:
  Vec3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0f;
  float eps = 1e-6f;

 private:
  // CollisionPipeline collision_pipeline;

 public:
  // void clear(ObjRegistry& registry);
  // void reset(ObjRegistry& registry);
  std::optional<ADMMError> step(ObjRegistry& registry, CudaRuntime rt);

 private:
  // Bbox compute_scene_bbox(ObjRegistry& registry);
  // void compute_barrier_constrain(const cu::buffer<float>& state,
  //                                const std::vector<Collision>& collisions,
  //                                BarrierConstrain& barrier);
};

}  // namespace silk::cuda
