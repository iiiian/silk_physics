#pragma once

#include <Eigen/Core>
#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk {
struct TriMesh;
}  // namespace silk

namespace silk::cuda {

class ObjectState {
 public:
  // The range of this object in the global state array.
  int state_offset = 0;
  int state_num = 0;

  cu::device_buffer<float> curr_state;
  cu::device_buffer<float> state_velocity;

  // Vertex permutation: new solver vertex index -> original mesh vertex index.
  cu::device_buffer<int> perm;
  // Inverse permutation: original mesh vertex index -> new solver vertex index.
  cu::device_buffer<int> inv_perm;

  ObjectState(int state_offset, ctd::span<const float> curr_state,
              ctd::span<const float> state_velocity, CudaRuntime rt);
  ObjectState(int state_offset, const ::silk::TriMesh& mesh);
};

}  // namespace silk::cuda
