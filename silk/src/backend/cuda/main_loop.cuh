#pragma once

#include <cuda/buffer>
#include <optional>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

class MainLoop {
 public:
  enum class Error { NothingToSolve, Diverge };

  Vec3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_inner_iteration = 100;
  int max_outer_iteration = 100;
  float dt = 1.0;
  // Lagrange multipler / penalty param for hard constraints.
  // float init_penalty = 1.0;
  float max_lagrange_mul = 1e8;
  // float max_penalty = 1e8;
  // float penalty_scaling_factor = 2;
  // float penalty_scaling_threshold = 0.25;
  // linear solver param
  float linear_rel_tol = 1e-3;
  // non-linear solver param
  float non_linear_rel_tol = 1e-3;
  float non_linear_abs_tol = 1e-20;
  // collision cache settings
  int init_broadphase_cache_size = 10000;
  int init_narrowphase_cache_size = 1000;

 public:
  // void reset(ObjRegistry& registry);
  std::optional<Error> step(ObjRegistry& registry, CudaRuntime rt);
};

}  // namespace silk::cuda
