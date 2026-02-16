#pragma once

#include <cassert>

#include "backend/cuda/collision/bbox.cuh"

namespace silk::cuda {

// Vector of colliding pairs
template <typename C>
using CollisionCache = std::vector<std::pair<C*, C*>>;

/// Return false to skip testing the pair
template <typename C>
using CollisionFilter = std::function<bool(const C&, const C&)>;

}  // namespace silk::cuda
