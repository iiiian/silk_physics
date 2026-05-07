#pragma once

#include <vector>

#include "backend/cuda/cuda_utils.cuh"
#include "common/mesh.hpp"

namespace silk::cuda {

class MeshPartition {
 public:
  /// Host vertex permutation form original order to patition sorted.
  /// Maps original vertex id -> permuted vertex id.
  std::vector<int> h_perm;
  /// Host inverse vertex permutation from partition sorted to original.
  /// Maps permuted vertex id -> original id.
  std::vector<int> h_inv_perm;
  /// Device vertex permutation form original order to patition sorted.
  /// Maps original vertex id -> permuted vertex id.
  Buf<int> d_perm;
  /// Device inverse vertex permutation from partition sorted to original.
  /// Maps permuted vertex id -> original id.
  Buf<int> d_inv_perm;
  /// Device CSR style partition offset after permutation.
  Buf<int> d_partition_offsets;

  MeshPartition() = default;
  MeshPartition(const TriMesh& mesh, CudaRuntime rt);

  /// @brief Permuted vertex position.
  ///
  /// Works on both device/host but not across.
  ///
  /// @param in Position in. Size == 3 * vertex num.
  /// @param out Position out. Size == 3 * vertex num.
  void permute(ctd::span<const float> in, ctd::span<float> out, CudaRuntime rt);

  /// @brief Inverse permuted vertex position.
  ///
  /// Works on both device/host but not across.
  ///
  /// @param in Position in. Size == 3 * vertex num.
  /// @param out Position out. Size == 3 * vertex num.
  void inv_permute(ctd::span<const float> in, ctd::span<float> out,
                   CudaRuntime rt);
};

}  // namespace silk::cuda
