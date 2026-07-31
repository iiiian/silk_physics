#pragma once

#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/stream>

#include "backend/cuda/cuda_utils.cuh"
#include "gpu_adapters.cuh"

namespace silk::broadphase_benchmark {

namespace cu = ::cuda;
namespace ctd = ::cuda::std;
using silk::cuda::check_cuda;

struct GpuContext {
  cu::device_ref device = cu::devices[0];
  cu::stream stream{device};
  cu::device_memory_pool memory_pool{device};

  silk::cuda::CudaRuntime runtime() {
    return {.stream = stream, .mr = memory_pool.as_ref()};
  }
};

}  // namespace silk::broadphase_benchmark
