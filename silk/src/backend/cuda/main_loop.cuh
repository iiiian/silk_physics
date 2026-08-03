#pragma once

#include <cuda/buffer>
#include <optional>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/admm_solver.cuh"

namespace silk::cuda {

class MainLoop {
 public:
  enum class Error { NothingToSolve, Diverge };

  Vec3f const_acceleration = {0.0f, 0.0f, -1.0f};
  int max_outer_iteration = 100;
  float dt = 1.0;
  // collision cache settings
  int init_broadphase_cache_size = 10000;
  int init_narrowphase_cache_size = 1000;
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  float ccd_minimum_separation_scale = 0.2f;
  float dcd_activation_distance_scale = 1.0f;

  ADMMSolver admm_solver;

 public:
  std::optional<Error> step(ObjRegistry& registry, CudaRuntime rt);
};

}  // namespace silk::cuda
