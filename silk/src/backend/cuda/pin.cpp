#include "backend/cuda/pin.hpp"

#include <cstring>

namespace silk::cuda {

PinIndex::PinIndex(std::span<const int> index)
    : index(index.begin(), index.end()) {}

PinPosition PinPosition::from_position(std::span<const float> position) {
  PinPosition p;
  p.curr_position.resize(position.size());
  memcpy(p.curr_position.data(), position.data(),
         position.size() * sizeof(float));
  p.prev_position = p.curr_position;

  return p;
}

PinPosition PinPosition::from_vertices(const PinIndex& index,
                                       std::span<const float> V) {
  if (index.is_all_pinned) {
    return from_position(V);
  }

  PinPosition p;
  p.curr_position.resize(3 * index.index.size());
  for (int i = 0; i < index.index.size(); ++i) {
    int vertex = index.index[i];
    for (int j = 0; j < 3; ++j) {
      p.curr_position[3 * i + j] = V[3 * vertex + j];
    }
  }
  p.prev_position = p.curr_position;

  return p;
}

}  // namespace silk::cuda
