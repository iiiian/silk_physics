#include "backend/cuda/graph_partition.hpp"

#include <ckaminpar.h>

#include <cassert>
#include <cstdint>
#include <sstream>
#include <thread>
#include <type_traits>
#include <vector>

namespace silk::cuda {

void graph_partition(ctd::span<int> row_ptr, ctd::span<int> cols,
                     ctd::span<int64_t> weights, int max_part_size,
                     int &part_num, std::vector<int> &part_id) {
  int node_num = row_ptr.size() - 1;
  assert(node_num >= 0);
  assert(max_part_size > 0);
  assert(weights.empty() || weights.size() == cols.size());

  part_id.resize(node_num, 0);
  if (node_num <= max_part_size) {
    part_num = 1;
    return;
  }

  static_assert(std::is_same_v<kaminpar_node_id_t, std::uint32_t>,
                "KaMinPar node IDs are expected to be uint32_t");
  static_assert(std::is_same_v<kaminpar_edge_id_t, std::uint32_t>,
                "KaMinPar edge IDs are expected to be uint32_t");
  static_assert(std::is_same_v<kaminpar_block_id_t, std::uint32_t>,
                "KaMinPar block IDs are expected to be uint32_t");
  static_assert(std::is_same_v<kaminpar_edge_weight_t, std::int64_t>,
                "KaMinPar edge weights are expected to be int64_t (enable "
                "KAMINPAR_64BIT_WEIGHTS)");
  static_assert(std::is_same_v<kaminpar_block_weight_t, std::int64_t>,
                "KaMinPar block weights are expected to be int64_t (enable "
                "KAMINPAR_64BIT_WEIGHTS)");

  std::vector<kaminpar_edge_id_t> xadj(row_ptr.size());
  for (size_t i = 0; i < row_ptr.size(); ++i) {
    if (row_ptr[i] < 0) {
      std::ostringstream oss;
      oss << "[MAS] Invalid graph for KaMinPar: row_ptr[" << i
          << "] is negative: " << row_ptr[i];
      throw std::runtime_error(oss.str());
    }
    xadj[i] = static_cast<kaminpar_edge_id_t>(row_ptr[i]);
  }

  std::vector<kaminpar_node_id_t> adjncy(cols.size());
  for (size_t i = 0; i < cols.size(); ++i) {
    if (cols[i] < 0) {
      std::ostringstream oss;
      oss << "[MAS] Invalid graph for KaMinPar: cols[" << i
          << "] is negative: " << cols[i];
      throw std::runtime_error(oss.str());
    }
    adjncy[i] = static_cast<kaminpar_node_id_t>(cols[i]);
  }

  auto *adjwgt = weights.empty()
                     ? nullptr
                     : reinterpret_cast<kaminpar_edge_weight_t *>(weights.data());

  kaminpar_context_t *ctx = kaminpar_create_context_by_preset_name("default");
  assert(ctx);

  int thread_num = (std::thread::hardware_concurrency() != 0)
                       ? std::thread::hardware_concurrency()
                       : 1;
  kaminpar_t *partitioner = kaminpar_create(thread_num, ctx);
  assert(partitioner);

  kaminpar_set_output_level(partitioner, KAMINPAR_OUTPUT_LEVEL_QUIET);
kaminpar_borrow_and_mutate_graph(partitioner,
                                   static_cast<kaminpar_node_id_t>(node_num),
                                   xadj.data(), adjncy.data(), nullptr,
                                   adjwgt);

  // Impl Eq.7 of arXiv:2411.06224

  // Forcing tight K for k-way algorithm leads to low quality parition.
  // It seems like max_part_size - 2 is a good balance.
  part_num = (node_num + max_part_size - 3) / (max_part_size - 2);
  assert(part_num >= 1);
  const auto k = static_cast<kaminpar_block_id_t>(part_num);
  std::vector<kaminpar_block_weight_t> max_block_weights(
      k, static_cast<kaminpar_block_weight_t>(max_part_size));
  std::vector<kaminpar_block_id_t> partition(node_num);

  // K way partition with absolution max block weight.
  // Since our node weight is uniform, this is equivalent to limiting parition
  // size.
  kaminpar_compute_partition_with_max_block_weights(partitioner, k,
                                                      max_block_weights.data(),
                                                      partition.data());

  kaminpar_context_free(ctx);
  kaminpar_free(partitioner);

  for (int i = 0; i < node_num; ++i) {
    if (partition[i] >= k) {
      std::ostringstream oss;
      oss << "[MAS] Invalid partition id from KaMinPar at node " << i << ": "
          << partition[i] << " not in [0, " << part_num << ")";
      throw std::runtime_error(oss.str());
    }
    part_id[i] = static_cast<int>(partition[i]);
  }
}

}  // namespace silk::cuda
