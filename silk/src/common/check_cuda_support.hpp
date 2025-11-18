#pragma once

namespace silk {

// Return true if CUDA backend is available in this build and at runtime.
// It considers both compile-time availability (SILK_WITH_CUDA) and a runtime
// query for device/runtime presence when compiled with CUDA.
bool check_cuda_support();

}  // namespace silk
