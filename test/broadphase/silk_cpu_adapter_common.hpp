#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

#include "backend/cpu/collision/bbox.hpp"
#include "backend/cpu/collision/broadphase.hpp"
#include "broadphase_common.hpp"

namespace silk::broadphase_benchmark {

struct SilkCollider {
  silk::cpu::Bbox bbox;
  std::array<int, 3> vertex_ids;
  int id = -1;
};

inline std::vector<SilkCollider> make_silk_colliders(
    std::span<const Box> boxes) {
  std::vector<SilkCollider> result;
  result.reserve(boxes.size());
  for (const Box& box : boxes) {
    result.push_back({{box.min, box.max}, box.vertex_ids, box.id});
  }
  return result;
}

struct CanCollide {
  bool operator()(const SilkCollider& a, const SilkCollider& b) const {
    for (int va : a.vertex_ids) {
      if (va < 0) {
        continue;
      }
      for (int vb : b.vertex_ids) {
        if (va == vb) {
          return false;
        }
      }
    }
    return true;
  }
};

inline silk::cpu::Bbox root_bbox(const std::vector<SilkCollider>& a,
                                 const std::vector<SilkCollider>& b = {}) {
  silk::cpu::Bbox result = a.front().bbox;
  for (const SilkCollider& collider : a) {
    result.merge_inplace(collider.bbox);
  }
  for (const SilkCollider& collider : b) {
    result.merge_inplace(collider.bbox);
  }
  for (int axis = 0; axis < 3; ++axis) {
    result.min(axis) = std::nextafter(result.min(axis),
                                      -std::numeric_limits<float>::infinity());
    result.max(axis) = std::nextafter(result.max(axis),
                                      std::numeric_limits<float>::infinity());
  }
  return result;
}

}  // namespace silk::broadphase_benchmark
