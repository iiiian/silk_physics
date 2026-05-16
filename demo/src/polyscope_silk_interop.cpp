#include "polyscope_silk_interop.hpp"

std::span<const float> make_const_span_from_position(
    ManagedBuffer<glm::vec3>& position) {
  static_assert(sizeof(glm::vec3) == 3 * sizeof(float));

  return std::span<const float>(
      reinterpret_cast<const float*>(position.data.data()),
      3 * position.size());
}

std::span<float> make_span_from_position(ManagedBuffer<glm::vec3>& position) {
  static_assert(sizeof(glm::vec3) == 3 * sizeof(float));

  return std::span<float>(
      reinterpret_cast<float*>(position.data.data()),
      3 * position.size());
}
