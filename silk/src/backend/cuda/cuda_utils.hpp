#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <exception>

#include "common/logger.hpp"

namespace silk::cuda {

// #define CUDA_ASSERT(val) check_throw_cuda((val), #val, __FILE__, __LINE__)
// void cuda_assert(cudaError_t result, char const *const func,
//                  const char *const file, int const line) {
//   assert(result == cudaSuccess && "CUDA call failed");
// }

#define CHECK_CUDA(val) check_cuda((val), #val, __FILE__, __LINE__)
inline void check_cuda(cudaError_t result, char const* const func,
                       const char* const file, int const line) {
  if (result != cudaSuccess) {
    SPDLOG_ERROR("CUDA Error at {}:{} code={} \"{}\" : {}", file, line,
                 static_cast<unsigned int>(result), func,
                 cudaGetErrorString(result));
    throw std::runtime_error("CUDA Error");
  }
}

#define CHECK_CUSPARSE(val) check_cusparse((val), #val, __FILE__, __LINE__)
inline void check_cusparse(cusparseStatus_t result, char const* const func,
                           const char* const file, int const line) {
  if (result != CUSPARSE_STATUS_SUCCESS) {
    SPDLOG_ERROR("cuSparse Error at {}:{} code={} \"{}\" : {}", file, line,
                 static_cast<unsigned int>(result), func,
                 cusparseGetErrorString(result));
    throw std::runtime_error("cuSparse Error");
  }
}

template <typename T>
T* host_vector_to_device(const T* vec, int num) {
  size_t size = num * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr;
  CUDA_CHECK(cudaMalloc(&ptr, size));
  CUDA_CHECK(cudaMemcpy(ptr, vec, size, cudaMemcpyHostToDevice));

  return ptr;
}

template <typename T>
T* host_vector_to_device(const std::vector<T>& vec) {
  size_t size = vec.size() * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr;
  CUDA_CHECK(cudaMalloc(&ptr, size));
  CUDA_CHECK(cudaMemcpy(ptr, vec.data(), size, cudaMemcpyHostToDevice));

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

  CUDA_CHECK(cudaMalloc(&d_ptr, bytes));
  CUDA_CHECK(cudaMemcpy(d_ptr, tmp.data(), bytes, cudaMemcpyHostToDevice));

  return d_ptr;
}

}  // namespace silk::cuda
