#pragma once

#include <polyscope/surface_mesh.h>

#include <silk/silk.hpp>

using polyscope::render::ManagedBuffer;

silk::ConstSpan<float> make_const_span_from_position(
    ManagedBuffer<glm::vec3>& position);

silk::Span<float> make_span_from_position(ManagedBuffer<glm::vec3>& position);
