#include "backend/cuda/pin.hpp"

#include <cstring>

namespace silk::cuda {

PinIndex::PinIndex(std::span<const int> index)
    : index(index.begin(), index.end()) {}

PinPosition::PinPosition(const RMatrixX3f& V) {
  curr_position.resize(3 * V.rows());
  std::memcpy(curr_position.data(), V.data(),
              curr_position.size() * sizeof(float));
  prev_position = curr_position;
}

PinPosition::PinPosition(const PinIndex& index, const RMatrixX3f& V) {
  if (index.is_all_pinned) {
    curr_position.resize(3 * V.rows());
    std::memcpy(curr_position.data(), V.data(),
                curr_position.size() * sizeof(float));
  } else {
    curr_position.resize(3 * index.index.size());
    for (int i = 0; i < index.index.size(); ++i) {
      int vertex = index.index[i];
      for (int j = 0; j < 3; ++j) {
        curr_position[3 * i + j] = V(vertex, j);
      }
    }
  }
  prev_position = curr_position;
}

PinPosition::PinPosition(const PinIndex& index, std::span<const float> V) {
  if (index.is_all_pinned) {
    curr_position.resize(V.size());
    std::memcpy(curr_position.data(), V.data(),
                curr_position.size() * sizeof(float));
  } else {
    curr_position.resize(3 * index.index.size());
    for (int i = 0; i < index.index.size(); ++i) {
      int vertex = index.index[i];
      for (int j = 0; j < 3; ++j) {
        curr_position[3 * i + j] = V[3 * vertex + j];
      }
    }
  }
  prev_position = curr_position;
}

}  // namespace silk::cuda
