#pragma once

#include <cuda.h>

#include <vector>

namespace silk::gpu {

template <typename T>
T* host_vector_to_device(const std::vector<T>& vec) {
  size_t size = vec.size() * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr;
  auto r = cudaMalloc(&ptr, size);
  if (r != cudaSuccess) {
    SPDLOG_DEBUG("Fail to allocate cuda memory. Reason {}.",
                 cudaGetErrorString(r));
    return nullptr;
  }

  r = cudaMemcpy(ptr, vec.data(), size, cudaMemcpyHostToDevice);
  if (r != cudaSuccess) {
    SPDLOG_DEBUG("Fail to copy vector data to device memory. Reason {}.",
                 cudaGetErrorString(r));
    return nullptr;
  }

  return ptr;
}

}  // namespace silk::gpu
