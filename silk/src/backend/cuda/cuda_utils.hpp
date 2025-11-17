#pragma once
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

}  // namespace silk::cuda
