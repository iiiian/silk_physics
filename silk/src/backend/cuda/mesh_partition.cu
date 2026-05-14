#include "backend/cuda/mesh_partition.cuh"

#include <cuda_runtime.h>

#include <Eigen/SparseCore>
#include <cassert>
#include <cuda/algorithm>
#include <vector>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/graph_partition.hpp"
#include "common/mesh.hpp"

namespace silk::cuda {

namespace {

bool is_device_mem(const void* ptr) {
  cudaPointerAttributes attr;
  CHECK_CUDA(cudaPointerGetAttributes(&attr, ptr));
  return (attr.type == cudaMemoryTypeHost);
}

void h_perm_then_assign(ctd::span<const float> in, ctd::span<const int> perm,
                        ctd::span<float> out) {
  for (int vid = 0; vid < perm.size(); ++vid) {
    int out_idx = perm[vid];
    for (int i = 0; i < 3; ++i) {
      out[3 * out_idx + i] = in[3 * vid + i];
    }
  }
}

__global__ void d_perm_then_assign(ctd::span<const float> in,
                                   ctd::span<const int> perm,
                                   ctd::span<float> out) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid > perm.size()) {
    return;
  }

  int out_idx = perm[tid];
#pragma unroll
  for (int i = 0; i < 3; ++i) {
    out[3 * out_idx + i] = in[3 * tid + i];
  }
}

}  // namespace

MeshPartition::MeshPartition(const TriMesh& mesh, CudaRuntime rt) {
  std::vector<Eigen::Triplet<float>> triplets;
  for (auto& e : mesh.E.rowwise()) {
    triplets.emplace_back(e(0), e(1), 0.0);
    triplets.emplace_back(e(1), e(0), 0.0);
  }

  int vert_num = mesh.V.rows();
  Eigen::SparseMatrix<float> adj(vert_num, vert_num);
  adj.setFromTriplets(triplets.begin(), triplets.end());

  int part_num;
  std::vector<int> part_id;

  ctd::span<int> row_ptr(adj.outerIndexPtr(), vert_num + 1);
  ctd::span<int> cols(adj.innerIndexPtr(), adj.nonZeros());
  // TODO: remove hard-coded 32 partition size.
  graph_partition(row_ptr, cols, {}, 32, part_num, part_id);

  // Count per partition vert num + prefix sum.
  h_partition_offsets = std::vector(part_num + 1, 0);
  for (int vid = 0; vid < vert_num; ++vid) {
    h_partition_offsets[part_id[vid] + 1] += 1;
  }
  for (int part = 0; part < part_num; ++part) {
    h_partition_offsets[part + 1] += h_partition_offsets[part];
  }

  // Build permutation.
  std::vector<int> tmp = h_partition_offsets;
  h_perm.resize(vert_num);
  for (int vid = 0; vid < vert_num; ++vid) {
    int part = part_id[vid];
    h_perm[vid] = tmp[part];
    tmp[part] += 1;
  }

  // Build inverse permutation.
  h_inv_perm.resize(vert_num);
  for (int vid = 0; vid < vert_num; ++vid) {
    h_inv_perm[h_perm[vid]] = vid;
  }

  d_perm = alloc<int>(rt, vert_num);
  d_inv_perm = alloc<int>(rt, vert_num);

  cu::copy_bytes(rt.stream, h_perm, *d_perm);
  cu::copy_bytes(rt.stream, h_inv_perm, *d_inv_perm);
  rt.stream.sync();
}

void MeshPartition::permute(ctd::span<const float> in, ctd::span<float> out,
                            CudaRuntime rt) {
  assert(in.size() == 3 * h_perm.size());
  assert(out.size() == 3 * h_perm.size());
  assert(is_device_mem(in.data()) == is_device_mem(out.data()));

  if (is_device_mem(in.data())) {
    int grid_num = div_round_up(d_perm->size(), 128);
    d_perm_then_assign<<<grid_num, 128, 0, rt.stream.get()>>>(in, *d_perm, out);
  } else {
    h_perm_then_assign(in, h_perm, out);
  }
}

void MeshPartition::inv_permute(ctd::span<const float> in, ctd::span<float> out,
                                CudaRuntime rt) {
  assert(in.size() == 3 * h_perm.size());
  assert(out.size() == 3 * h_perm.size());
  assert(is_device_mem(in.data()) == is_device_mem(out.data()));

  if (is_device_mem(in.data())) {
    int grid_num = div_round_up(d_perm->size(), 128);
    d_perm_then_assign<<<grid_num, 128, 0, rt.stream.get()>>>(in, *d_inv_perm,
                                                              out);
  } else {
    h_perm_then_assign(in, h_inv_perm, out);
  }
}

};  // namespace silk::cuda
