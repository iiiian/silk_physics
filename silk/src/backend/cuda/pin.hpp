#pragma once

#include <span>
#include <vector>

#include "common/eigen_alias.hpp"

namespace silk::cuda {

struct PinIndex {
  bool is_all_pinned = false;
  std::vector<int> index;

  PinIndex() = default;
  explicit PinIndex(std::span<const int> index);
};

struct PinPosition {
  bool is_static = true;
  bool is_static_twice = true;
  std::vector<float> curr_position;
  std::vector<float> prev_position;

  PinPosition() = default;
  explicit PinPosition(const RMatrixX3f& V);
  PinPosition(const PinIndex& index, const RMatrixX3f& V);
  PinPosition(const PinIndex& index, std::span<const float> V);
};

}  // namespace silk::cuda
