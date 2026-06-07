#include "backend/cuda/graph_partition.hpp"

#include <cassert>
#include <vector>

#if defined(SILK_GRAPH_PARTITION_BACKEND_KAMINPAR)

#include <ckaminpar.h>

#include <thread>

namespace silk::cuda {

void graph_partition(ctd::span<int> row_ptr, ctd::span<int> cols,
                     ctd::span<int> weights, int max_part_size, int &part_num,
                     std::vector<int> &part_id) {
  int node_num = row_ptr.size() - 1;
  assert(node_num >= 0);
  assert(max_part_size > 0);
  assert(weights.empty() || weights.size() == cols.size());

  part_id.resize(node_num, 0);
  if (node_num <= max_part_size) {
    part_num = 1;
    return;
  }

  std::vector<kaminpar_edge_id_t> xadj(row_ptr.size());
  for (size_t i = 0; i < row_ptr.size(); ++i) {
    xadj[i] = static_cast<kaminpar_edge_id_t>(row_ptr[i]);
  }

  std::vector<kaminpar_node_id_t> adjncy(cols.size());
  for (size_t i = 0; i < cols.size(); ++i) {
    adjncy[i] = static_cast<kaminpar_node_id_t>(cols[i]);
  }

  std::vector<kaminpar_edge_weight_t> adjwgt;
  if (!weights.empty()) {
    adjwgt.resize(weights.size());
    for (int i = 0; i < weights.size(); ++i) {
      adjwgt[i] = static_cast<kaminpar_edge_weight_t>(weights[i]);
    }
  }

  kaminpar_context_t *ctx = kaminpar_create_context_by_preset_name("default");
  assert(ctx);

  int thread_num = (std::thread::hardware_concurrency() != 0)
                       ? std::thread::hardware_concurrency()
                       : 1;
  kaminpar_t *partitioner = kaminpar_create(thread_num, ctx);
  assert(partitioner);

  kaminpar_set_output_level(partitioner, KAMINPAR_OUTPUT_LEVEL_QUIET);
  kaminpar_borrow_and_mutate_graph(
      partitioner, static_cast<kaminpar_node_id_t>(node_num), xadj.data(),
      adjncy.data(), nullptr, adjwgt.data());

  // Forcing tight K for k-way algorithm leads to low quality parition.
  // It seems like max_part_size - 2 is a good balance.
  part_num = (node_num + max_part_size - 3) / (max_part_size - 2);
  assert(part_num >= 1);
  const auto k = static_cast<kaminpar_block_id_t>(part_num);
  std::vector<kaminpar_block_weight_t> max_block_weights(
      k, static_cast<kaminpar_block_weight_t>(max_part_size));
  std::vector<kaminpar_block_id_t> partition(node_num);

  kaminpar_compute_partition_with_max_block_weights(
      partitioner, k, max_block_weights.data(), partition.data());

  kaminpar_context_free(ctx);
  kaminpar_free(partitioner);

  for (int i = 0; i < node_num; ++i) {
    part_id[i] = static_cast<int>(partition[i]);
  }

  // Kaminpar might return empty parition.
  // Remap to filter that out.
  std::vector<int> remap(part_num, -1);
  int non_empty_part_num = 0;
  for (int i = 0; i < node_num; ++i) {
    int p = part_id[i];
    if (remap[p] == -1) {
      remap[p] = non_empty_part_num;
      ++non_empty_part_num;
    }
    part_id[i] = remap[p];
  }
  part_num = non_empty_part_num;
}

}  // namespace silk::cuda

#elif defined(SILK_GRAPH_PARTITION_BACKEND_METIS)

#define IDXTYPEWIDTH 32
#define REALTYPEWIDTH 32
#include <metis.h>
#undef IDXTYPEWIDTH
#undef REALTYPEWIDTH

#include <stdexcept>

namespace silk::cuda {

void graph_partition(ctd::span<int> row_ptr, ctd::span<int> cols,
                     ctd::span<int> weights, int max_part_size, int &part_num,
                     std::vector<int> &part_id) {
  int node_num = row_ptr.size() - 1;
  assert(node_num >= 0);
  assert(max_part_size > 0);
  assert(weights.empty() || weights.size() == cols.size());

  part_id.resize(node_num, 0);
  if (node_num <= max_part_size) {
    part_num = 1;
    return;
  }

  static_assert(sizeof(int) == 4, "int is not 32 bits");
  const idx_t *xadj = reinterpret_cast<const idx_t *>(row_ptr.data());
  const idx_t *adjncy = reinterpret_cast<const idx_t *>(cols.data());
  const idx_t *adjwgt = weights.empty()
                            ? nullptr
                            : reinterpret_cast<const idx_t *>(weights.data());

  for (int slack = 1; slack < max_part_size; ++slack) {
    int ideal_part_size = max_part_size - slack;
    part_num = (node_num + ideal_part_size - 1) / ideal_part_size;
    assert(part_num >= 1);

    idx_t ncon = 1;
    idx_t objval = 0;
    idx_t nvtxs = node_num;
    idx_t nparts = part_num;
    part_id.resize(node_num, 0);
    int ret = METIS_PartGraphKway(
        &nvtxs, &ncon, const_cast<idx_t *>(xadj), const_cast<idx_t *>(adjncy),
        nullptr, nullptr, const_cast<idx_t *>(adjwgt), &nparts, nullptr,
        nullptr, nullptr, &objval, reinterpret_cast<idx_t *>(part_id.data()));
    if (ret != METIS_OK) {
      throw std::runtime_error("[graph_partition] METIS partition failed.");
    }

    std::vector<int> part_size(part_num, 0);
    int current_max = 0;
    for (int i = 0; i < node_num; ++i) {
      int p = part_id[i];
      assert(p >= 0 && p < part_num);
      part_size[p] += 1;
      current_max = std::max(current_max, part_size[p]);
    }

    if (current_max <= max_part_size) {
      return;
    }
  }

  throw std::runtime_error(
      "[graph_partition] METIS partition size constraint failed.");
}

}  // namespace silk::cuda

#endif
