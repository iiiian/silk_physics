#pragma once

#include <polyscope/surface_mesh.h>

#include <silk/silk.hpp>
#include <span>

using polyscope::render::ManagedBuffer;

std::span<const float> make_const_span_from_position(
    ManagedBuffer<glm::vec3>& position);

std::span<float> make_span_from_position(ManagedBuffer<glm::vec3>& position);
