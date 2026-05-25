#pragma once

#include <span>
#include <vector>

namespace silk::cuda {

class PinIndex {
 public:
  bool is_all_pinned = false;
  std::vector<int> index;

  PinIndex() = default;
  explicit PinIndex(std::span<const int> index);
};

class PinPosition {
 public:
  bool is_static = true;
  bool is_static_twice = true;
  std::vector<float> curr_position;
  std::vector<float> prev_position;

  PinPosition() = default;
  static PinPosition from_position(std::span<const float> position);
  static PinPosition from_vertices(const PinIndex& index,
                                   std::span<const float> V);
};

}  // namespace silk::cuda
