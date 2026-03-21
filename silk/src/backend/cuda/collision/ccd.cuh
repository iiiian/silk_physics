#pragma once

#include <cuda/std/numeric>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

struct CubicPolyRoot {
  static constexpr float EMPTY = ctd::numeric_limits<float>::max();

  float a;
  float b;
  float c;
};

__both__ CubicPolyRoot solve_coplaner_poly(Vec3fV x1_t0, Vec3fV x2_t0,
                                           Vec3fV x3_t0, Vec3fV x4_t0,
                                           Vec3fV x1_t1, Vec3fV x2_t1,
                                           Vec3fV x3_t1, Vec3fV x4_t1);

}  // namespace silk::cuda
