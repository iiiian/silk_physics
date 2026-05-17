#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <cassert>
#include <cuda/algorithm>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/memory_resource>
#include <cuda/std/source_location>
#include <cuda/std/span>
#include <cuda/stream>
#include <format>
#include <stdexcept>
#include <string>
#include <vector>

#include "backend/cuda/namespace_alias.hpp"

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

struct CudaRuntime {
  cu::stream_ref stream;
  cu::mr::resource_ref<cu::mr::device_accessible> mr;
};

class CudaOOM : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// @brief ceil(num/denom)
constexpr int div_round_up(int num, int denom) {
  return (num + denom - 1) / denom;
}

/// Throw meaningful error if allocation fails.
template <typename T, typename V = ::cuda::no_init_t>
cu::device_buffer<T> alloc(
    CudaRuntime rt, size_t n, V init_val = cu::no_init,
    ctd::source_location location = ctd::source_location::current()) {
  size_t requested_bytes = n * sizeof(T);
  // Only catch cu::cuda_error with status() = cudaErrorMemoryAllocation
  try {
    return cu::make_buffer<T>(rt.stream, rt.mr, n, init_val);
  } catch (const CudaOOM&) {
    throw;
  } catch (const cu::cuda_error& err) {
    if (err.status() != cudaErrorMemoryAllocation) {
      throw;
    }
  }

  size_t free_bytes = 0;
  size_t total_bytes = 0;
  cudaMemGetInfo(&free_bytes, &total_bytes);
  constexpr size_t MB = 1024ull * 1024ull;
  size_t requested_mb = div_round_up(requested_bytes, MB);
  size_t free_mb = div_round_up(free_bytes, MB);
  throw CudaOOM(
      std::format("Fail to alllocate {} MB at {}: line {} {}. Only {} MB left.",
                  requested_mb, location.file_name(), location.line(),
                  location.function_name(), free_mb));
}

/// @brief Nullable device buffer.
/// It's very annoying device_buffer does not have default ctor.
template <typename T>
using Buf = ctd::optional<cu::device_buffer<T>>;

template <typename T>
struct DynSpan {
  // please respect the fill counter with atomic ops.
  int* fill = nullptr;
  ctd::span<T> data;
};

template <typename T>
void resize_buffer(size_t size, cu::device_buffer<T>& buf, CudaRuntime rt) {
  auto new_buf = alloc<T>(rt, size);
  cu::copy_bytes(rt.stream, buf, ctd::span{new_buf.data(), buf.size()});
  buf = new_buf;
}

template <typename T>
cu::device_buffer<T> vec_like_to_device(ctd::span<const T> vec,
                                        CudaRuntime rt) {
  auto buffer = alloc<T>(rt, vec.size());
  cu::copy_bytes(rt.stream, vec, buffer);
  return buffer;
}

template <typename T>
cu::device_buffer<T> vec_like_to_device(std::span<const T> vec,
                                        CudaRuntime rt) {
  auto buffer = alloc<T>(rt, vec.size());
  cu::copy_bytes(rt.stream, vec, buffer);
  return buffer;
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
