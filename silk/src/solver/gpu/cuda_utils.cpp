#include "logger.hpp"

namespace silk::gpu {
#define CUDA_THROW(val) check_throw_cuda((val), #val, __FILE__, __LINE__)
void check_throw_cuda(cudaError_t result, char const *const func,
                      const char *const file, int const line) {
  assert(result == cudaSuccess && "CUDA call failed");
}

#define CHECK_CUDA_ERROR(val) check_cuda((val), #val, __FILE__, __LINE__)
void check_cuda(cudaError_t result, char const *const func,
                const char *const file, int const line) {
  if (result != cudaSuccess) {
    SPDLOG_ERROR("CUDA Error at {}:{} code={} \"{}\" : {}", file, line,
                 static_cast<unsigned int>(result), func,
                 cudaGetErrorString(result));
    throw std::runtime_error("CUDA Error");
  }
}
}  // namespace silk::gpu