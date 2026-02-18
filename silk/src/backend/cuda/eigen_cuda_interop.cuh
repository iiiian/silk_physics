#pragma once

#include <Eigen/Core>
#include <cassert>
#include <cuda/buffer>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

template <typename Derived>
cu::device_buffer<typename Derived::Scalar> host_eigen_to_device(
    const Eigen::DenseBase<Derived>& expr, CudaRuntime rt) {
  assert(expr.size() > 0);

  // Ensure we have a contiguous temporary on the host
  // even if expr is a block/segment/other expression.
  const auto tmp = expr.derived().eval();
  size_t num = static_cast<size_t>(expr.size());

  return cu::make_buffer(rt.stream, rt.mem_resource, {tmp.data(), num});
}

}  // namespace silk::cuda
