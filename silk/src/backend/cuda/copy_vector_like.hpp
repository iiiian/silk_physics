#pragma once

#include <Eigen/Core>
#include <vector>

#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

template <typename T>
T* host_vector_to_device(const T* vec, int num) {
  size_t size = num * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr = nullptr;
  CHECK_CUDA(cudaMalloc((void**)&ptr, size));
  CHECK_CUDA(cudaMemcpy(ptr, vec, size, cudaMemcpyHostToDevice));

  return ptr;
}

template <typename T>
T* host_vector_to_device(const std::vector<T>& vec) {
  size_t size = vec.size() * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr = nullptr;
  CHECK_CUDA(cudaMalloc((void**)&ptr, size));
  CHECK_CUDA(cudaMemcpy(ptr, vec.data(), size, cudaMemcpyHostToDevice));

  return ptr;
}

template <typename Derived>
auto host_eigen_to_device(const Eigen::DenseBase<Derived>& expr) ->
    typename Derived::Scalar* {
  using Scalar = typename Derived::Scalar;

  size_t n = static_cast<size_t>(expr.size());
  if (n == 0) {
    return nullptr;
  }

  // Ensure we have a contiguous temporary on the host
  // even if expr is a block/segment/other expression.
  const auto tmp = expr.derived().eval();  // PlainObject, contiguous

  Scalar* d_ptr = nullptr;
  size_t bytes = n * sizeof(Scalar);

  CHECK_CUDA(cudaMalloc((void**)&d_ptr, bytes));
  CHECK_CUDA(cudaMemcpy(d_ptr, tmp.data(), bytes, cudaMemcpyHostToDevice));

  return d_ptr;
}

}  // namespace silk::cuda
