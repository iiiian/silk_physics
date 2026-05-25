#include "backend/cuda/solver/inner_product.cuh"

#include <cassert>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/std/span>

namespace silk::cuda {

namespace {

__global__ void inner_product_kernel(ctd::span<const float> a,
                                     ctd::span<const float> b, float *out) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;

  float c = (tid < a.size()) ? a[tid] * b[tid] : 0.0f;

  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ typename BlockReduce::TempStorage tmp;

  float reduced = BlockReduce(tmp).Sum(c);

  if (threadIdx.x == 0) {
    cu::atomic_ref<float> a_out{*out};
    a_out.fetch_add(reduced, ctd::memory_order_relaxed);
  }
}
}  // namespace

void inner_product(ctd::span<const float> a, ctd::span<const float> b,
                   ctd::span<float> out, CudaRuntime rt) {
  assert(a.size() == b.size());
  assert(out.size() == 1);

  cu::fill_bytes(rt.stream, out, 0);
  int grid_num = div_round_up(a.size(), 128);
  inner_product_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(a, b, out.data());
}

}  // namespace silk::cuda
