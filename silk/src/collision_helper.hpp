#pragma once

#include <functional>

#include "bbox.hpp"

namespace silk {

template <typename T>
struct BboxCollider {
  Bbox bbox;
  bool is_static;
  T data;
};

template <typename T>
using CollisionCache = std::vector<std::pair<T, T>>;

template <typename T>
using CollisionFilterCallback = std::function<bool(const T&, const T&)>;

}  // namespace silk
