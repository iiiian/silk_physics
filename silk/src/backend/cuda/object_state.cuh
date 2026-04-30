#pragma once

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

  Buf<float> curr_state;
  Buf<float> state_velocity;

  // Vertex permutation: new solver vertex index -> original mesh vertex index.
  Buf<int> perm;
  // Inverse permutation: original mesh vertex index -> new solver vertex index.
  Buf<int> inv_perm;

  ObjectState(int state_offset, ctd::span<const float> curr_state,
              ctd::span<const float> state_velocity, CudaRuntime rt);
  ObjectState(int state_offset, const ::silk::TriMesh& mesh);
};

}  // namespace silk::cuda
