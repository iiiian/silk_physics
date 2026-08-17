#pragma once

#include <cuda/buffer>
#include <cuda/std/span>
#include <utility>
#include <vector>

#include "backend/cuda/solver/contact_constraints.cuh"

namespace silk::cuda {

/// Color contacts that share movable vertices and return CSR-style color
/// groups as contact indices and group offsets.
std::pair<cu::device_buffer<int>, std::vector<int>> color_contacts(
    ctd::span<const FrictionContact> contacts,
    ctd::span<const int> vertex_offsets, ctd::span<const int> incident_contacts,
    CudaRuntime rt);

}  // namespace silk::cuda
