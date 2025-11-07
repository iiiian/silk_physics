#include "collision/gpu/bvh_context.hpp"

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <cassert>
#include <vector>

namespace silk::gpu {

template <typename T>
T* host_vector_to_device(const std::vector<T>& vec) {
  size_t size = vec.size() * sizeof(T);
  if (size == 0) {
    return nullptr;
  }

  T* ptr;
  auto r = cudaMalloc(&ptr, size);
  if (r != cudaSuccess) {
    SPDLOG_DEBUG("Fail to allocate cuda memory. Reason {}.",
                 cudaGetErrorString(r));
    return nullptr;
  }

  r = cudaMemcpy(ptr, vec.data(), size, cudaMemcpyHostToDevice);
  if (r != cudaSuccess) {
    SPDLOG_DEBUG("Fail to copy vector data to device memory. Reason {}.",
                 cudaGetErrorString(r));
    return nullptr;
  }

  return ptr;
}

BVHContext BVHContext::make_bvh_context(
    const std::vector<cuBQL::box3f>& edge_bboxes,
    const std::vector<EdgeCollider>& edge_colliders,
    const std::vector<cuBQL::box3f>& triangle_bboxes,
    const std::vector<TriangleCollider>& triangle_colliders) {
  assert(edge_bboxes.size()==edge_colliders.size());
  assert(triangle_bboxes.size()==triangle_colliders.size());

  BVHContext c;

  c.edge_num=edge_bboxes.size();
  c.d_edge_bboxes = host_vector_to_device(edge_bboxes);
  assert(c.d_edge_bboxes);
  c.d_edge_colliders = host_vector_to_device(edge_colliders);
  assert(c.d_edge_colliders);

  c.triangle_num=triangle_bboxes.size();
  c.d_triangle_bboxes = host_vector_to_device(triangle_bboxes);
  assert(c.d_triangle_bboxes);
  c.d_triangle_colliders = host_vector_to_device(triangle_colliders);
  assert(c.d_triangle_colliders);

  return c;
}

BVHContext::~BVHContext(){
  if (d_edge_bboxes){
    cudaFree(d_edge_bboxes);
  }
  if (d_edge_colliders) {
    cudaFree(d_edge_colliders);
  }
  if (d_triangle_bboxes) {
    cudaFree(d_triangle_bboxes);
  }
  if (d_triangle_colliders) {
    cudaFree(d_triangle_colliders);
  }
}

}  // namespace silk::gpu
