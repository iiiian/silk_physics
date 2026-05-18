#pragma once

#include <algorithm>
#include <vector>

#include "config.hpp"
#include "eigen_alias.hpp"

namespace pin_selection {

inline std::vector<int> select_bbox_vertices(const Vert& verts,
                                             const config::PinBBox& bbox) {
  std::array<float, 3> bmin = bbox.min;
  std::array<float, 3> bmax = bbox.max;
  for (int axis = 0; axis < 3; ++axis) {
    if (bmin[axis] > bmax[axis]) {
      std::swap(bmin[axis], bmax[axis]);
    }
  }

  std::vector<int> indices;
  for (int i = 0; i < verts.rows(); ++i) {
    bool in_bbox = true;
    for (int axis = 0; axis < 3; ++axis) {
      float x = verts(i, axis);
      if (x < bmin[axis] || x > bmax[axis]) {
        in_bbox = false;
        break;
      }
    }
    if (in_bbox) {
      indices.push_back(i);
    }
  }
  return indices;
}

}  // namespace pin_selection
