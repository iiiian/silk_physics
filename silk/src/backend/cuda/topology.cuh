#pragma once

#include <backend/cuda/cuda_utils.cuh>
#include <cuda/buffer>

namespace silk::cuda {

struct Topology {
  cu::device_buffer<int> V;
  cu::device_buffer<int> E;
};

}  // namespace silk::cuda
