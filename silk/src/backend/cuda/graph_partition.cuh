#pragma once

#include <cuda/std/span>
#include <cstdint>
#include <vector>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {
    /// @brief K-way graph partition.
    /// @param row_ptr[in] CSR graph topology. Will be modified!
    /// @param cols[in] CSR graph topology. Will be modified!
    /// @param weights[in] CSR graph edge weights. Empty span implies unweighted graph. Will be modified!
    /// @param max_part_size[in] Maximum partition size.
    /// @param part_num[out] Total partition num.
    /// @param part_id[out] Partition id of each graph node.
    void graph_partition(ctd::span<int> row_ptr,
                         ctd::span<int> cols,
                         ctd::span<int64_t> weights,
                         int max_part_size,
                         int &part_num,
                         std::vector<int> &part_id);
} // namespace silk::cuda
