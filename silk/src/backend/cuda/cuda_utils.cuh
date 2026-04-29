#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <cuda/algorithm>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/memory_resource>
#include <cuda/std/span>
#include <cuda/stream>
#include <stdexcept>
#include <string>
#include <vector>

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

/// @brief ceil(num/denom)
constexpr int div_round_up(int num, int denom) {
  return (num + denom - 1) / denom;
}

struct CudaRuntime {
  cu::stream_ref stream;
  cu::mr::resource_ref<cu::mr::device_accessible> mr;
};

/// @brief Nullable device buffer.
/// It's very annoying device_buffer does not have default ctor.
template <typename T>
using Buf = ctd::optional<cu::device_buffer<T>>;

template <typename T>
struct DynSpan {
  // please respect the counter with atomic ops.
  int* counter = nullptr;
  ctd::span<T> data;
};

template <typename T>
void resize_buffer(size_t size, cu::device_buffer<T>& buf, CudaRuntime rt) {
  auto new_buf = cu::make_buffer(rt.stream, rt.mr, size, cu::no_init);
  cu::copy_bytes(rt.stream, buf, ctd::span{new_buf.data(), buf.size()});
  buf = new_buf;
}

template <typename T>
cu::device_buffer<T> vec_like_to_device(ctd::span<const T> vec,
                                        CudaRuntime rt) {
  auto buffer = cu::make_buffer<T>(rt.stream, rt.mr, vec.size(), cu::no_init);
  cu::copy_bytes(rt.stream, vec, buffer);
}

template <typename T>
std::vector<T> vec_like_to_host(ctd::span<const T> vec, CudaRuntime rt) {
  std::vector<T> buffer(vec.size());
  cu::copy_bytes(rt.stream, vec, buffer);
}

template <typename T>
T scalar_load(const T* device_ptr, CudaRuntime rt) {
  T result;
  cudaMemcpyAsync(&result, device_ptr, sizeof(T), cudaMemcpyDeviceToHost,
                  rt.stream.get());
  rt.stream.sync();
  return result;
}

template <typename T>
void scalar_write(T* dst, T val, CudaRuntime rt) {
  cudaMemcpyAsync(dst, &val, sizeof(T), cudaMemcpyHostToDevice,
                  rt.stream.get());
}

}  // namespace silk::cuda
