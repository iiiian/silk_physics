#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <stdexcept>
#include <string>

namespace silk::cuda {

#define CHECK_CUDA(val) check_cuda((val), #val, __FILE__, __LINE__)
inline void check_cuda(cudaError_t result, char const* const func,
                       const char* const file, int const line) {
  if (result != cudaSuccess) {
    std::string msg;
    msg.reserve(128);  // optional

    msg += "CUDA Error at ";
    msg += file;
    msg += ":";
    msg += std::to_string(line);
    msg += " code=";
    msg += std::to_string(static_cast<unsigned int>(result));
    msg += " \"";
    msg += func;
    msg += "\" : ";
    msg += cudaGetErrorString(result);

    throw std::runtime_error(msg);
  }
}

#define CHECK_CUSPARSE(val) check_cusparse((val), #val, __FILE__, __LINE__)
inline void check_cusparse(cusparseStatus_t result, char const* const func,
                           const char* const file, int const line) {
  if (result != CUSPARSE_STATUS_SUCCESS) {
    std::string msg;
    msg.reserve(128);  // optional

    msg += "cuSparse Error at ";
    msg += file;
    msg += ":";
    msg += std::to_string(line);
    msg += " code=";
    msg += std::to_string(static_cast<unsigned int>(result));
    msg += " \"";
    msg += func;
    msg += "\" : ";
    msg += cusparseGetErrorString(result);

    throw std::runtime_error(msg);
  }
}

}  // namespace silk::cuda
