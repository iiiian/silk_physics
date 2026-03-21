#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/memory_resource>
#include <cuda/stream>
#include <stdexcept>
#include <string>

// libcu++ cuda:: and cuda::std:: namespace conflicts with silk::cuda,
// making it annoying to use. To remedy this, rename them to cu:: and ctd::.
namespace cuda {}
namespace cu = ::cuda;
namespace cuda::std {}
namespace ctd = ::cuda::std;

namespace silk::cuda {

#define __both__ __host__ __device__

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

/// @brief Compute grid num given minimal thread num and a fixed block dim.
constexpr int compute_grid_num(int thread_num, int block_dim) {
  return (thread_num + block_dim - 1) / block_dim;
}

struct CudaRuntime {
  cu::stream_ref stream;
  cu::mr::resource_ref<cu::mr::device_accessible> mem_resource;
};

template <typename T>
struct AtomicBuffer {
  cu::atomic<int> counter;
  cu::device_buffer<T> data;

  AtomicBuffer(int num, CudaRuntime rt) {
    counter = 0;
    data = cu::make_buffer(rt.stream, rt.mem_resource, num, cu::no_init);
  }
};

}  // namespace silk::cuda
