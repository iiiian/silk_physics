#pragma once

#include <cuda_runtime_api.h>

#include <utility>

#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

template <typename T>
class DVector {
 public:
  T* data = nullptr;

 public:
  DVector() = default;

  DVector(size_t size) {
    CHECK_CUDA(cudaMalloc((void**)&data, size * sizeof(T)));
  }

  DVector(const DVector& other) = delete;

  DVector(DVector&& other) noexcept { swap(other); }

  DVector& operator=(const DVector& other) = delete;

  DVector& operator=(DVector&& other) noexcept {
    swap(other);
    return *this;
  }

  ~DVector() { CHECK_CUDA(cudaFree(data)); }

  operator T*() const { return data; }

 private:
  void swap(DVector& other) noexcept {
    if (this == &other) {
      return;
    }
    std::swap(data, other.data);
  }
};

}  // namespace silk::cuda
