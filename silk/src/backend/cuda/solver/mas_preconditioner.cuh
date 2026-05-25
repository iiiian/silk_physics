#pragma once

#include <cuda/std/array>

#include "backend/cuda/bsr_matrix.cuh"
#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

// Our bank size is 32, so 4 coarse levels should cover 1M x 1M BSR matrix.
constexpr int MAS_MAX_COARSE_LEVEL = 4;
constexpr int MAS_LEVEL_NUM = MAS_MAX_COARSE_LEVEL + 1;

// Topology enhanced MAS has 2 levels of mapping:
// Real -> Padded: virtual nodes are inserted so each partition resides in one
// BANK.
//                 see fig.6 in arXiv:2411.06224.
// Padded -> Coarse Space: map fine nodes to coarse space CCO (Connected
// components).

/// @brief Real <-> Padded space mapping.
struct PaddedTopology {
  Buf<int> real_to_padded;  ///< Real id to padded id.
  Buf<int> padded_to_real;  ///< Padded id to real id. -1 = virtual padding.
  Buf<int> rows;            ///< CSR row ptr. Virtual nodes are not included.
  Buf<int> cols;            ///< CSR cols. Virtual nodes are not included.
  int node_num = 0;         ///< Real node num.
  int padded_node_num = 0;  ///< Padded space node num.
};

struct CoarseSpace {
  /// Coarse space map (real id -> coarse space id) layout:
  /// | node0_lv0 ... node0_lv_max | node1_lv0 ... node1_lv_max | ... |
  Buf<int> map;
  /// CCO numbers per coarse space level.
  ctd::array<int, MAS_MAX_COARSE_LEVEL> cco_nums{};
  /// Total coarse space levels. Sometimes we terminates early cause there are
  /// no CCO to merge.
  int level_num = 0;
};

struct CoarseMatrices {
  /// Packed upper triangular row major coarse space matrices (inverse hessian).
  /// | lv0_mat0 lv0_mat1 ... | lv1_mat0 ... | ... |
  Buf<float> data;
  /// Per coarse space level offset and num.
  ctd::array<int, MAS_LEVEL_NUM> matrix_offsets{};
  ctd::array<int, MAS_LEVEL_NUM> matrix_nums{};
  /// We support block dim 1,2,3. So coarse space mat dim should be 32,64,96.
  int mat_dim = 0;
  int mat_storage_size = 0;
  int total_matrix_num = 0;
};

struct CoarseVectors {
  /// | lv0 r | lv 1 r | ... |
  Buf<float> multi_level_r;
  /// | lv0 z | lv 1 z | ... |
  Buf<float> multi_level_z;
  /// Per coarse space level offset and num.
  ctd::array<int, MAS_LEVEL_NUM> level_offsets{};
  ctd::array<int, MAS_LEVEL_NUM> level_nums{};
  int total_level_nodes = 0;
};

class MASPreconditioner {
 public:
  /// @brief Initialize MAS preconditioner. Build coarse space, compute inverse,
  /// allocate buffer...
  /// @param A Input matrix and scalar diagonal update sorted by graph
  /// partition.
  /// @param part_offsets Graph paritition offset. CSR row ptr style.
  /// @param rt Cuda runtime.
  void factorize(DynamicBSRView A, ctd::span<const int> part_offsets,
                 CudaRuntime rt);

  /// @brief Apply MAS preconditioner.
  /// @param r Residual.
  /// @param z Preconditioned residual.
  /// @param rt Cuda runtime.
  void apply(ctd::span<const float> r, ctd::span<float> z, CudaRuntime rt);

 private:
  int block_dim_ = 0;
  PaddedTopology padded_topology_;
  CoarseSpace coarse_space_;
  CoarseMatrices coarse_matrices_;
  CoarseVectors coarse_vectors_;
};

}  // namespace silk::cuda
