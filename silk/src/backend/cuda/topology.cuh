#pragma once

#include <cuda/buffer>

#include "backend/cuda/namespace_alias.hpp"

namespace silk::cuda {

struct Topology {
  cu::device_buffer<int> V;
  cu::device_buffer<int> E;
};

}  // namespace silk::cuda
