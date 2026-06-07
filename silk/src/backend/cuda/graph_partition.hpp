#pragma once

#include <cuda/std/span>
#include <vector>

#include "backend/cuda/namespace_alias.hpp"

namespace silk::cuda {

/// @brief K-way graph partition.
/// @param row_ptr CSR graph topology. Will be modified!
/// @param cols CSR graph topology. Will be modified!
/// @param weights CSR graph edge weights. Empty span implies unweighted
/// graph. Will be modified!
/// @param max_part_size Maximum partition size.
/// @param part_num Total partition num.
/// @param part_id Partition id of each graph node.
void graph_partition(ctd::span<int> row_ptr, ctd::span<int> cols,
                     ctd::span<int> weights, int max_part_size, int &part_num,
                     std::vector<int> &part_id);

}  // namespace silk::cuda
