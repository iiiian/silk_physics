#include "common/check_cuda_support.hpp"

#include "common/logger.hpp"

#ifdef SILK_WITH_CUDA
#include <cuda_runtime_api.h>
#endif

namespace silk {

bool check_cuda_support() {
#ifdef SILK_WITH_CUDA
  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  if (err != cudaSuccess || device_count == 0) {
    SPDLOG_ERROR("No runtime cuda support");
    return false;
  }
  SPDLOG_INFO("Cuda support OK.");
  return true;
#else
  SPDLOG_ERROR("Not compiled with Cuda support.");
  return false;
#endif
}

}  // namespace silk
