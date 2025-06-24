#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "common_types.hpp"

class Mesh {
 public:
  RMatrixX3f V;
  RMatrixX3i F;

  Mesh(const RMatrixX3f& V, const RMatrixX3i& F);
  Mesh(float* vertices, uint32_t vert_num, int* faces, uint32_t face_num);
  bool is_valid() const;
};
