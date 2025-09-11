#include "polyscope_silk_interop.hpp"

silk::ConstSpan<float> make_const_span_from_position(
    ManagedBuffer<glm::vec3>& position) {
  static_assert(sizeof(glm::vec3) == 3 * sizeof(float));

  silk::ConstSpan<float> s;
  s.data = reinterpret_cast<const float*>(position.data.data());
  // Number of floats = 3 * number of vec3 elements
  s.size = 3 * static_cast<int>(position.size());

  return s;
}

silk::Span<float> make_span_from_position(ManagedBuffer<glm::vec3>& position) {
  static_assert(sizeof(glm::vec3) == 3 * sizeof(float));

  silk::Span<float> s;
  s.data = reinterpret_cast<float*>(position.data.data());
  // Number of floats = 3 * number of vec3 elements
  s.size = 3 * static_cast<int>(position.size());

  return s;
}
