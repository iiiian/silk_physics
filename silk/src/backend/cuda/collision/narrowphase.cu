#include <cuda/atomic>
#include <cuda/buffer>
#include <cuda/std/span>
#include <cuda/std/utility>

#include "backend/cuda/collision/ccd.cuh"
#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/collision/mesh_collider.cuh"
#include "backend/cuda/collision/narrowphase.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda::collision {

namespace {

__global__ void batch_pt_ccd(ctd::span<PTCCache> pt_ccache,
                             float minimal_seperation, DynSpan<Collision> out) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid > pt_ccache.size()) {
    return;
  }

  auto [t, p] = pt_ccache[tid];
  auto c = pt_ccd(p, t, minimal_seperation);
  if (!c) {
    return;
  }

  // Skip if out buffer is full.
  cu::atomic_ref<int> fill{*out.fill};
  int out_idx = fill.fetch_add(1);
  if (out_idx < out.data.size()) {
    out.data[out_idx] = *c;
  }
}

__global__ void batch_ee_ccd(ctd::span<EECCache> ee_ccache,
                             float minimal_seperation, DynSpan<Collision> out) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid > ee_ccache.size()) {
    return;
  }

  auto [ea, eb] = ee_ccache[tid];
  auto c = ee_ccd(ea, eb, minimal_seperation);
  if (!c) {
    return;
  }

  // Skip if out buffer is full.
  cu::atomic_ref<int> fill{*out.fill};
  int out_idx = fill.fetch_add(1);
  if (out_idx < out.data.size()) {
    out.data[out_idx] = *c;
  }
}

}  // namespace

void pt_narrowphase(ctd::span<PTCCache> pt_ccache, float minimal_seperation,
                    cu::device_buffer<Collision>& out, int& fill,
                    CudaRuntime rt) {
  if (pt_ccache.empty()) {
    return;
  }

  auto d_fill = cu::make_buffer<int>(rt.stream, rt.mr, 1, fill);
  DynSpan<Collision> dyn_out{.fill = d_fill.data(), .data = out};

  int grid_num = div_round_up(pt_ccache.size(), 128);
  batch_pt_ccd<<<grid_num, 128, 0, rt.stream.get()>>>(
      pt_ccache, minimal_seperation, dyn_out);

  // If buffer overlfow, resize then try again.
  int old_fill = fill;
  fill = scalar_load(d_fill.data(), rt);
  if (fill > out.size()) {
    resize_buffer(fill + 1, out, rt);
    dyn_out.data = out;
    scalar_write(d_fill.data(), old_fill, rt);
    batch_pt_ccd<<<grid_num, 128, 0, rt.stream.get()>>>(
        pt_ccache, minimal_seperation, dyn_out);
  }
}

void ee_narrowphase(ctd::span<EECCache> ee_ccache, float minimal_seperation,
                    float restitution, float friction,
                    cu::device_buffer<Collision>& out, int& fill,
                    CudaRuntime rt) {
  if (ee_ccache.empty()) {
    return;
  }

  auto d_fill = cu::make_buffer<int>(rt.stream, rt.mr, 1, fill);
  DynSpan<Collision> dyn_out{.fill = d_fill.data(), .data = out};

  int grid_num = div_round_up(ee_ccache.size(), 128);
  batch_ee_ccd<<<grid_num, 128, 0, rt.stream.get()>>>(
      ee_ccache, minimal_seperation, dyn_out);

  // If buffer overlfow, resize then try again.
  int old_fill = fill;
  fill = scalar_load(d_fill.data(), rt);
  if (fill > out.size()) {
    resize_buffer(fill + 1, out, rt);
    dyn_out.data = out;
    scalar_write(d_fill.data(), old_fill, rt);
    batch_ee_ccd<<<grid_num, 128, 0, rt.stream.get()>>>(
        ee_ccache, minimal_seperation, dyn_out);
  }
}

}  // namespace silk::cuda::collision
