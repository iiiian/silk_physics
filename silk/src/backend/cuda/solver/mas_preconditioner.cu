#include "backend/cuda/solver/mas_preconditioner.cuh"

#include <spdlog/spdlog.h>

#include <cassert>
#include <chrono>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/array>
#include <cuda/std/bit>
#include <cuda/std/utility>
#include <cuda/warp>
#include <stdexcept>

#include "common/compiler_builtin.hpp"

namespace silk::cuda {

namespace {

using clock = std::chrono::steady_clock;

[[maybe_unused]] float elapsed(const std::chrono::time_point<clock> &begin) {
  return std::chrono::duration<float>(clock::now() - begin).count();
}

// This is not tunable.
constexpr int BANK_SIZE = 32;

/// @brief Convenience wrapper for CoarseMatrices.
struct CoarseMatricesRef {
  int mat_dim;
  int mat_storage_size;
  ctd::array<ctd::span<float>, MAS_LEVEL_NUM> matrix_per_level;

  CoarseMatricesRef(CoarseMatrices &mats) {
    mat_dim = mats.mat_dim;
    mat_storage_size = mats.mat_storage_size;

    int scalar_offset = 0;
    for (int i = 0; i < MAS_LEVEL_NUM; ++i) {
      int level_size = mats.matrix_nums[i] * mats.mat_storage_size;
      matrix_per_level[i] =
          ctd::span<float>(mats.data->data() + scalar_offset, level_size);
      scalar_offset += level_size;
    }
  }
};

__global__ void build_padded_maps(ctd::span<const int> part_offsets,
                                  ctd::span<const int> bsr_rows,
                                  ctd::span<int> real_to_padded,
                                  ctd::span<int> padded_to_real,
                                  ctd::span<int> real_num_per_row) {
  int padded_id = blockDim.x * blockIdx.x + threadIdx.x;
  if (padded_id >= real_num_per_row.size()) {
    return;
  }

  int part = padded_id / BANK_SIZE;
  int local = padded_id % BANK_SIZE;
  int part_begin = part_offsets[part];
  int part_end = part_offsets[part + 1];
  int part_size = part_end - part_begin;

  // Real nodes.
  if (local < part_size) {
    int real_id = part_begin + local;
    real_to_padded[real_id] = padded_id;
    padded_to_real[padded_id] = real_id;
    real_num_per_row[padded_id] = bsr_rows[real_id + 1] - bsr_rows[real_id];
  }
  // Virtual padding.
  else {
    padded_to_real[padded_id] = -1;
    real_num_per_row[padded_id] = 0;
  }
}

__global__ void fill_padded_cols(ctd::span<const int> bsr_rows,
                                 ctd::span<const int> bsr_cols,
                                 ctd::span<const int> real_to_padded,
                                 ctd::span<const int> padded_rows,
                                 ctd::span<int> padded_cols) {
  int real_id = blockDim.x * blockIdx.x + threadIdx.x;
  int node_num = bsr_rows.size() - 1;
  if (real_id >= node_num) {
    return;
  }

  int padded_id = real_to_padded[real_id];
  int dst = padded_rows[padded_id];
  for (int n = bsr_rows[real_id]; n < bsr_rows[real_id + 1]; ++n) {
    padded_cols[dst] = real_to_padded[bsr_cols[n]];
    ++dst;
  }
}

PaddedTopology build_padded_topology(BSRView A,
                                     ctd::span<const int> part_offsets,
                                     CudaRuntime rt) {
  int node_num = A.dim;
  int part_num = part_offsets.size() - 1;
  int padded_node_num = part_num * BANK_SIZE;

  PaddedTopology topo;
  topo.node_num = node_num;
  topo.padded_node_num = padded_node_num;
  topo.real_to_padded = alloc<int>(rt, node_num, -1);
  topo.padded_to_real = alloc<int>(rt, padded_node_num, -1);
  topo.rows = alloc<int>(rt, padded_node_num + 1);
  topo.cols = alloc<int>(rt, A.non_zeros);

  auto real_num_per_row = alloc<int>(rt, padded_node_num, 0);
  int grid_num = div_round_up(padded_node_num, 128);
  build_padded_maps<<<grid_num, 128, 0, rt.stream.get()>>>(
      part_offsets, A.rows, *(topo.real_to_padded), *(topo.padded_to_real),
      real_num_per_row);

  // Build padded space row_ptr.
  size_t cub_tmp_size = 0;
  // clang-format off
  cub::DeviceScan::ExclusiveSum(nullptr,
                                cub_tmp_size,
                                real_num_per_row.data(),
                                topo.rows->data(),
                                padded_node_num,
                                rt.stream.get());
  auto cub_tmp = alloc<char>(rt, cub_tmp_size);
  cub::DeviceScan::ExclusiveSum(cub_tmp.data(),
                                cub_tmp_size,
                                real_num_per_row.data(),
                                topo.rows->data(),
                                padded_node_num,
                                rt.stream.get());
  // clang-format on

  scalar_write(topo.rows->data() + padded_node_num, A.non_zeros, rt);
  grid_num = div_round_up(node_num, 128);
  fill_padded_cols<<<grid_num, 128, 0, rt.stream.get()>>>(
      A.rows, A.cols, *(topo.real_to_padded), *(topo.rows), *(topo.cols));

  return topo;
}

/// @brief Get coarse space CCO id.
/// @param map Coarse space map.
/// @param vid Vertex id.
/// @param level Coarse space level. Start at lv 0.
/// @return CCO id.
__both__ int get_coarse_space_id(ctd::span<const int> map, int vid, int level) {
  return map[vid * MAS_MAX_COARSE_LEVEL + level];
}

/// @brief Build local CCO mapping from input space -> coarse space lv 0 and
/// collapse topology.
/// @param row_ptr CSR graph topology (may include self).
/// @param cols CSR graph topology (may include self).
/// @param padded_to_real Padded id -> real id. -1 denotes padding.
/// @param cco_num_per_bank CCO number per bank.
/// @param local_cco_ids Bank local CCO id at coarse space lv 0.
__global__ void build_local_cco_lv0(ctd::span<const int> row_ptr,
                                    ctd::span<int> cols,
                                    ctd::span<const int> padded_to_real,
                                    ctd::span<int> cco_num_per_bank,
                                    ctd::span<int> local_cco_ids) {
  // Bank local neighbor masks.
  __shared__ uint32_t neighbors[128];

  int btid = threadIdx.x;                    // block local thread id
  int tid = blockDim.x * blockIdx.x + btid;  // global thread id
  int wid = threadIdx.x / BANK_SIZE;         // block local warp id
  int lid = threadIdx.x % BANK_SIZE;         // lane id
  int bid = tid / BANK_SIZE;                 // bank id

  if (lid == 0 && bid < cco_num_per_bank.size()) {
    cco_num_per_bank[bid] = 0;
  }

  int node_num = row_ptr.size() - 1;
  if (tid >= node_num) {
    return;
  }
  // Skip virtual padding.
  if (padded_to_real[tid] == -1) {
    local_cco_ids[tid] = -1;
    return;
  }

  // Build neighbor mask (including self) and collapse topology.
  int row_begin = row_ptr[tid];
  int row_end = row_ptr[tid + 1];
  uint32_t neighbor = (1u << lid);
  int out_of_bank_neighbor_num = 0;
  for (int n = row_begin; n < row_end; ++n) {
    int neighbor_vid = cols[n];
    int neighbor_bid = neighbor_vid / BANK_SIZE;
    if (bid == neighbor_bid) {
      neighbor |= (1u << (neighbor_vid % BANK_SIZE));
    } else {
      // Compact topology to remove in-bank neighbors so we can skip
      // additional filtering when building future coarse spaces.
      cols[row_begin + out_of_bank_neighbor_num] = neighbor_vid;
      ++out_of_bank_neighbor_num;
    }
  }
  if (row_begin + out_of_bank_neighbor_num < row_end) {
    // -1 denotes neighbor list ending.
    cols[row_begin + out_of_bank_neighbor_num] = -1;
  }
  neighbors[btid] = neighbor;
  __syncwarp();

  // Build connectivity mask (including self).
  uint32_t connection = neighbor;
  uint32_t visited = (1u << lid);
  uint32_t to_visit = connection ^ visited;
  while (to_visit) {
    int visiting = ctd::countr_zero(to_visit);
    connection |= neighbors[visiting + wid * BANK_SIZE];
    visited |= (1u << visiting);
    to_visit = connection ^ visited;
  }

  // Find bank (warp) local connected component (CCO).
  uint32_t cco_lead_lid = ctd::countr_zero(connection);
  bool is_lead = (cco_lead_lid == lid);
  uint32_t lead_lanes = __ballot_sync(0xFFFFFFFFu, is_lead);
  uint32_t before_cco_lead_mask =
      static_cast<uint32_t>((1ull << cco_lead_lid) - 1ull);

  // Write cco info back.
  local_cco_ids[tid] = ctd::popcount(lead_lanes & before_cco_lead_mask);
  if (lid == 0) {
    cco_num_per_bank[bid] = ctd::popcount(lead_lanes);
  }
}

/// @brief Build bank neighbor mask and collapse topology.
/// @param level Target coarse space level.
/// @param coarse_space_map Coarse space map.
/// @param row_ptr CSR graph topology (may include self).
/// @param cols CSR graph topology (may include self).
/// @param padded_to_real Padded id -> real id. -1 denotes padding.
/// @param neighbors Bank local neighbor mask.
__global__ void build_neighbor_masks_lvx(int level,
                                         ctd::span<const int> coarse_space_map,
                                         ctd::span<const int> row_ptr,
                                         ctd::span<int> cols,
                                         ctd::span<const int> padded_to_real,
                                         ctd::span<uint32_t> neighbors) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;  // global thread id
  int node_num = row_ptr.size() - 1;
  if (tid >= node_num) {
    return;
  }
  // Skip virtual padding.
  if (padded_to_real[tid] == -1) {
    return;
  }

  // Build neighbor mask (including self) and collapse topology.
  int row_begin = row_ptr[tid];
  int row_end = row_ptr[tid + 1];
  int real_id = padded_to_real[tid];
  int cco_id = get_coarse_space_id(coarse_space_map, real_id, level - 1);
  uint32_t neighbor = (1u << (cco_id % BANK_SIZE));
  int out_of_bank_neighbor_num = 0;
  for (int n = row_begin; n < row_end; ++n) {
    int neighbor_vid = cols[n];
    if (neighbor_vid == -1) {
      break;
    }

    int neighbor_real_id = padded_to_real[neighbor_vid];
    int neighbor_cco_id =
        get_coarse_space_id(coarse_space_map, neighbor_real_id, level - 1);
    if (cco_id / BANK_SIZE == neighbor_cco_id / BANK_SIZE) {
      neighbor |= (1u << (neighbor_cco_id % BANK_SIZE));
    } else {
      cols[row_begin + out_of_bank_neighbor_num] = neighbor_vid;
      ++out_of_bank_neighbor_num;
    }
  }
  if (row_begin + out_of_bank_neighbor_num < row_end) {
    cols[row_begin + out_of_bank_neighbor_num] = -1;
  }

  atomicOr(neighbors.data() + cco_id, neighbor);
}

/// @brief Build local CCO mapping from coarse space level x-1 -> coarse space
/// level x.
/// @param cco_num Total CCO number at coarse space lv x-1.
/// @param cco_num_per_bank Independent CCO number per bank at coarse space lv
/// x.
/// @param local_cco_ids Bank local CCO id at coarse space lv x.
__global__ void build_local_cco_lvx(int cco_num,
                                    ctd::span<const uint32_t> neighbors,
                                    ctd::span<int> cco_num_per_bank,
                                    ctd::span<int> local_cco_ids) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;  // global thread id
  int bid = tid / BANK_SIZE;                        // bank id
  int lid = tid % BANK_SIZE;                        // lane id
  if (tid >= cco_num) {
    return;
  }

  // Build connectivity mask (including self).
  uint32_t connection = neighbors[tid];
  uint32_t visited = (1u << lid);
  uint32_t to_visit = connection ^ visited;
  while (to_visit) {
    int visiting = ctd::countr_zero(to_visit);
    connection |= neighbors[visiting + bid * BANK_SIZE];
    visited |= (1u << visiting);
    to_visit = connection ^ visited;
  }

  // Find bank (warp) local connected component (CCO).
  uint32_t cco_lead_lid = ctd::countr_zero(connection);
  bool is_lead = (cco_lead_lid == lid);
  uint32_t lead_lanes = __ballot_sync(0xFFFFFFFFu, is_lead);
  uint32_t before_cco_lead_mask =
      static_cast<uint32_t>((1ull << cco_lead_lid) - 1ull);

  // Write cco info back.
  local_cco_ids[tid] = ctd::popcount(lead_lanes & before_cco_lead_mask);
  if (lid == 0) {
    cco_num_per_bank[bid] = ctd::popcount(lead_lanes);
  }
}

/// @brief Compute global CCO id and write it to coarse space map.
///
/// Compute global CCO id offset from prefix sum, then write result to
/// coarse_space_map.
__global__ void aggregate_coarse_space_map_lv0(
    ctd::span<const int> local_cco_ids, ctd::span<const int> cco_num_per_bank,
    ctd::span<const int> cco_num_per_bank_summed,
    ctd::span<const int> real_to_padded, ctd::span<int> coarse_space_map) {
  int real_id = blockDim.x * blockIdx.x + threadIdx.x;
  if (real_id >= real_to_padded.size()) {
    return;
  }

  int padded_id = real_to_padded[real_id];
  int bid = padded_id / BANK_SIZE;
  int cco_id = local_cco_ids[padded_id] + cco_num_per_bank_summed[bid] -
               cco_num_per_bank[bid];
  coarse_space_map[real_id * MAS_MAX_COARSE_LEVEL] = cco_id;
}

/// @brief Compute global CCO id and write it to coarse space map.
///
/// Compute global CCO id offset from prefix sum, then write result to
/// coarse_space_map.
__global__ void aggregate_coarse_space_map_lvx(
    int level, ctd::span<const int> local_cco_ids,
    ctd::span<const int> cco_num_per_bank,
    ctd::span<const int> cco_num_per_bank_summed,
    ctd::span<int> coarse_space_map) {
  int real_id = blockDim.x * blockIdx.x + threadIdx.x;
  int node_num = coarse_space_map.size() / MAS_MAX_COARSE_LEVEL;
  if (real_id >= node_num) {
    return;
  }

  int prev_cco_id = get_coarse_space_id(coarse_space_map, real_id, level - 1);
  int bid = prev_cco_id / BANK_SIZE;
  int cco_id = local_cco_ids[prev_cco_id] + cco_num_per_bank_summed[bid] -
               cco_num_per_bank[bid];
  coarse_space_map[real_id * MAS_MAX_COARSE_LEVEL + level] = cco_id;
}

/// @brief Build coarse space map.
///
/// Coarse space map real node id layout:
/// | node0_lv0 node0_lv1 ... node0_lv_max | node1_lv0 ... node1_lv_max | ... |
///
/// @param row_ptr CSR graph topology (may include self).
/// @param cols CSR graph topology (may include self).
/// @param real_to_padded Real id -> padded id.
/// @param padded_to_real Padded id -> real id. -1 denotes virtual padding.
/// @rt Cuda runtime config.
/// @warning Will modify cols as side effect!
CoarseSpace build_coarse_space(ctd::span<const int> row_ptr,
                               ctd::span<int> cols,
                               ctd::span<const int> real_to_padded,
                               ctd::span<const int> padded_to_real,
                               CudaRuntime rt) {
  ctd::array<int, MAS_MAX_COARSE_LEVEL> cco_nums{};

  int padded_node_num = row_ptr.size() - 1;
  int real_node_num = real_to_padded.size();
  int max_bank_per_level = div_round_up(padded_node_num, BANK_SIZE);
  // Bank local CCO id.
  auto local_cco_ids = alloc<int>(rt, padded_node_num);
  auto cco_num_per_bank = alloc<int>(rt, max_bank_per_level);
  // Inclusive prefix sum of cco_num_per_bank.
  auto cco_num_per_bank_summed = alloc<int>(rt, max_bank_per_level);
  auto coarse_space_map = alloc<int>(rt, real_node_num * MAS_MAX_COARSE_LEVEL);

  // Build coarse map level 0.
  int grid_num = div_round_up(padded_node_num, 128);
  build_local_cco_lv0<<<grid_num, 128, 0, rt.stream.get()>>>(
      row_ptr, cols, padded_to_real, cco_num_per_bank, local_cco_ids);

  // Prefix sum to compute global CCO id offset.
  size_t cub_tmp_size = 0;
  cub::DeviceScan::InclusiveSum(nullptr, cub_tmp_size, cco_num_per_bank.data(),
                                cco_num_per_bank_summed.data(),
                                max_bank_per_level, rt.stream.get());
  auto cub_tmp = alloc<char>(rt, cub_tmp_size);
  cub::DeviceScan::InclusiveSum(
      cub_tmp.data(), cub_tmp_size, cco_num_per_bank.data(),
      cco_num_per_bank_summed.data(), max_bank_per_level, rt.stream.get());

  grid_num = div_round_up(real_node_num, 128);
  aggregate_coarse_space_map_lv0<<<grid_num, 128, 0, rt.stream.get()>>>(
      local_cco_ids, cco_num_per_bank, cco_num_per_bank_summed, real_to_padded,
      coarse_space_map);

  int cco_num =
      scalar_load(cco_num_per_bank_summed.data() + max_bank_per_level - 1, rt);
  cco_nums[0] = cco_num;
  int level_num = 1;

  // Build coarse map level 1 ... MAS_MAX_COARSE_LEVEL-1 recursively.
  // We do:
  //  1. Build bank local neighbbor bit mask.
  //  2. Build bank local connectivity mask and find local CCO partition.
  //  3. Compute global CCO id and write to coarse space map.
  auto neighbors = alloc<uint32_t>(rt, padded_node_num);
  for (int lv = 1; lv < MAS_MAX_COARSE_LEVEL; ++lv) {
    cudaMemsetAsync(neighbors.data(), 0, cco_num * sizeof(uint32_t),
                    rt.stream.get());

    grid_num = div_round_up(padded_node_num, 128);
    build_neighbor_masks_lvx<<<grid_num, 128, 0, rt.stream.get()>>>(
        lv, coarse_space_map, row_ptr, cols, padded_to_real,
        ctd::span<uint32_t>(neighbors.data(), cco_num));

    int bank_num = div_round_up(cco_num, BANK_SIZE);
    grid_num = div_round_up(cco_num, 128);
    build_local_cco_lvx<<<grid_num, 128, 0, rt.stream.get()>>>(
        cco_num, ctd::span<const uint32_t>(neighbors.data(), cco_num),
        ctd::span<int>(cco_num_per_bank.data(), bank_num),
        ctd::span<int>(local_cco_ids.data(), cco_num));

    cub::DeviceScan::InclusiveSum(
        cub_tmp.data(), cub_tmp_size, cco_num_per_bank.data(),
        cco_num_per_bank_summed.data(), bank_num, rt.stream.get());
    grid_num = div_round_up(real_node_num, 128);
    aggregate_coarse_space_map_lvx<<<grid_num, 128, 0, rt.stream.get()>>>(
        lv, ctd::span<const int>(local_cco_ids.data(), cco_num),
        ctd::span<const int>(cco_num_per_bank.data(), bank_num),
        ctd::span<const int>(cco_num_per_bank_summed.data(), bank_num),
        coarse_space_map);

    // No more CCO to merge.
    int next_cco_num =
        scalar_load(cco_num_per_bank_summed.data() + bank_num - 1, rt);
    if (next_cco_num == cco_num) {
      break;
    }

    cco_num = next_cco_num;
    cco_nums[lv] = cco_num;
    level_num = lv + 1;
  }

  CoarseSpace cs;
  cs.cco_nums = cco_nums;
  cs.level_num = level_num;
  cs.map = std::move(coarse_space_map);
  return cs;
}

/// @brief Compute offset of row major upper triangular dim x dim matrix.
__both__ int index_upper_mat(int dim, int i, int j) {
  if (i > j) {
    ctd::swap(i, j);
  }
  return i * dim - (i * (i + 1) / 2) + j;
}

/// @brief Build coarse space matrices by accumulating padded space hessian.
__global__ void fill_coarse_matrices(DynamicBSRView mat_in,
                                     CoarseMatricesRef mat_out,
                                     ctd::span<const int> coarse_space_map,
                                     ctd::span<const int> real_to_padded,
                                     int coarse_level_num) {
  BSRView mat = mat_in.mat;
  int tid = blockDim.x * blockIdx.x + threadIdx.x;  // global thread id
  if (tid >= mat.dim) {
    return;
  }

  int padded_i = real_to_padded[tid];
  int block_size = mat.block_dim * mat.block_dim;
  for (int nz = mat.rows[tid]; nz < mat.rows[tid + 1]; ++nz) {
    int j = mat.cols[nz];
    // Assume input mat is symmetric (Which is required by CG anyway). Skip
    // lower.
    if (tid > j) {
      continue;
    }

    int padded_j = real_to_padded[j];
    int block_offset = nz * block_size;
    // Padded space matrices.
    if (padded_i / BANK_SIZE == padded_j / BANK_SIZE) {
      int mat_id = padded_i / BANK_SIZE;
      int scalar_i_root = (padded_i % BANK_SIZE) * mat.block_dim;
      int scalar_j_root = (padded_j % BANK_SIZE) * mat.block_dim;
      float *out_mat = mat_out.matrix_per_level[0].data() +
                       mat_id * mat_out.mat_storage_size;
      for (int bi = 0; bi < mat.block_dim; ++bi) {
        for (int bj = 0; bj < mat.block_dim; ++bj) {
          int row = scalar_i_root + bi;
          int col = scalar_j_root + bj;
          // Write upper part of coarse matrix only.
          if (row > col) {
            continue;
          }
          // For diagonal block (tid == j), write upper part only.
          if (tid == j && bi > bj) {
            continue;
          }

          float val = mat.vals[block_offset + bi * mat.block_dim + bj];
          atomicAdd(out_mat + index_upper_mat(mat_out.mat_dim, row, col), val);
        }
      }
    }
    // Coarse space matrices.
    for (int lv = 0; lv < coarse_level_num; ++lv) {
      int cco_i = coarse_space_map[tid * MAS_MAX_COARSE_LEVEL + lv];
      int cco_j = coarse_space_map[j * MAS_MAX_COARSE_LEVEL + lv];
      if (cco_i / BANK_SIZE != cco_j / BANK_SIZE) {
        continue;
      }

      int mat_id = cco_i / BANK_SIZE;
      int scalar_i_root = (cco_i % BANK_SIZE) * mat.block_dim;
      int scalar_j_root = (cco_j % BANK_SIZE) * mat.block_dim;
      float *out_mat = mat_out.matrix_per_level[lv + 1].data() +
                       mat_id * mat_out.mat_storage_size;
      for (int bi = 0; bi < mat.block_dim; ++bi) {
        for (int bj = 0; bj < mat.block_dim; ++bj) {
          int row = scalar_i_root + bi;
          int col = scalar_j_root + bj;
          // Write upper part of coarse matrix only.
          if (row > col) {
            continue;
          }
          // For diagonal block (tid == j), write upper part only.
          if (tid == j && bi > bj) {
            continue;
          }

          float val = mat.vals[block_offset + bi * mat.block_dim + bj];
          // When two different fine nodes collapse to the same coarse node,
          // the coarse diagonal block must receive both A_ij and A_ji = A_ij^T.
          if (tid != j && cco_i == cco_j) {
            val += mat.vals[block_offset + bj * mat.block_dim + bi];
          }
          atomicAdd(out_mat + index_upper_mat(mat_out.mat_dim, row, col), val);
        }
      }
    }
  }

  // Handle diagonal update.
  for (int bi = 0; bi < mat.block_dim; ++bi) {
    int scalar_id = tid * mat.block_dim + bi;
    if (scalar_id >= mat_in.diag.size()) {
      continue;
    }

    float val = mat_in.diag[scalar_id];
    int mat_id = padded_i / BANK_SIZE;
    int row = (padded_i % BANK_SIZE) * mat.block_dim + bi;
    float *padded_mat =
        mat_out.matrix_per_level[0].data() + mat_id * mat_out.mat_storage_size;
    atomicAdd(padded_mat + index_upper_mat(mat_out.mat_dim, row, row), val);

    for (int lv = 0; lv < coarse_level_num; ++lv) {
      int cco_id = coarse_space_map[tid * MAS_MAX_COARSE_LEVEL + lv];
      mat_id = cco_id / BANK_SIZE;
      row = (cco_id % BANK_SIZE) * mat.block_dim + bi;
      float *coarse_mat = mat_out.matrix_per_level[lv + 1].data() +
                          mat_id * mat_out.mat_storage_size;
      atomicAdd(coarse_mat + index_upper_mat(mat_out.mat_dim, row, row), val);
    }
  }
}

template <int BLOCK_DIM>
__global__ void gather_multi_level_r(
    ctd::span<const float> r, ctd::span<float> multi_level_r,
    ctd::span<const int> padded_to_real, ctd::span<const int> coarse_space_map,
    ctd::array<int, MAS_LEVEL_NUM> level_offsets, int coarse_level_num) {
  using WarpReduce = cub::WarpReduce<float, BANK_SIZE>;
  constexpr int WARPS_PER_BLOCK = 128 / BANK_SIZE;

  __shared__ typename WarpReduce::TempStorage
      reduce_storage[WARPS_PER_BLOCK][MAS_MAX_COARSE_LEVEL][BLOCK_DIM];

  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= padded_to_real.size()) {
    return;
  }

  int lid = threadIdx.x % BANK_SIZE;
  int wid = threadIdx.x / BANK_SIZE;
  int rid = padded_to_real[tid];

  if (rid < 0) {
    return;
  }

  float r_value[BLOCK_DIM];
#pragma unroll
  for (int i = 0; i < BLOCK_DIM; ++i) {
    r_value[i] = r[rid * BLOCK_DIM + i];
  }

#pragma unroll
  for (int i = 0; i < BLOCK_DIM; ++i) {
    multi_level_r[tid * BLOCK_DIM + i] = r_value[i];
  }

  for (int lv = 0; lv < coarse_level_num; ++lv) {
    int cco_id = coarse_space_map[rid * MAS_MAX_COARSE_LEVEL + lv];
    int prev_cco_id = cu::device::warp_shuffle_up<BANK_SIZE>(cco_id, 1).data;
    bool head = lid == 0 || cco_id != prev_cco_id;
    int dst_root = (level_offsets[lv + 1] + cco_id) * BLOCK_DIM;
#pragma unroll
    for (int i = 0; i < BLOCK_DIM; ++i) {
      float segment_sum = WarpReduce(reduce_storage[wid][lv][i])
                              .HeadSegmentedSum(r_value[i], head);
      if (head) {
        atomicAdd(multi_level_r.data() + dst_root + i, segment_sum);
      }
    }
  }
}

/// @brief Matrix vector multiplication. A is packed upper row major matrix.
///
/// This is the main bottleneck of MAS preconditioner. But since we are vram
/// bandwidth bound, there's nothing I could do. The compute / memory ratio is
/// just too low.
template <int N, int BLOCK>
__global__ void symv_upper_packed(const float *A_upper, const float *x,
                                  float *y) {
  int mat_id = blockIdx.x;
  int row = threadIdx.x;
  constexpr int L = N * (N + 1) / 2;
  const float *Amat = A_upper + mat_id * L;

  // All threads in block load A and x into shared mem.

  __shared__ float sx[N];
  __shared__ float sA[L];

  for (int i = threadIdx.x; i < N; i += BLOCK) {
    sx[i] = x[mat_id * N + i];
  }
  for (int k = threadIdx.x; k < L; k += BLOCK) {
    sA[k] = Amat[k];
  }

  __syncthreads();

  // Each thread do one row * x.

  if (row >= N) {
    return;
  }

  float sum = 0.0f;
  for (int col = 0; col < N; ++col) {
    sum += sA[index_upper_mat(N, row, col)] * sx[col];
  }

  y[mat_id * N + row] = sum;
}

void apply_inverse(const CoarseMatrices &mats, ctd::span<const float> x,
                   ctd::span<float> y, int block_dim, CudaRuntime rt) {
  int mat_num = mats.total_matrix_num;
  if (mat_num == 0) {
    return;
  }

  if (block_dim == 1) {
    symv_upper_packed<32, 32><<<mat_num, 32, 0, rt.stream.get()>>>(
        mats.data->data(), x.data(), y.data());
    return;
  }
  if (block_dim == 2) {
    symv_upper_packed<64, 64><<<mat_num, 64, 0, rt.stream.get()>>>(
        mats.data->data(), x.data(), y.data());
    return;
  }
  if (block_dim == 3) {
    symv_upper_packed<96, 96><<<mat_num, 96, 0, rt.stream.get()>>>(
        mats.data->data(), x.data(), y.data());
    return;
  }
}

__global__ void gather_multi_level_z(
    ctd::span<const float> multi_level_z, ctd::span<float> z,
    ctd::span<const int> real_to_padded, ctd::span<const int> coarse_space_map,
    ctd::array<int, MAS_LEVEL_NUM> level_offsets, int block_dim,
    int coarse_level_num) {
  int real_id = blockDim.x * blockIdx.x + threadIdx.x;
  if (real_id >= real_to_padded.size()) {
    return;
  }

  // Padded space.
  int padded_id = real_to_padded[real_id];
  int dst_root = real_id * block_dim;
  for (int i = 0; i < block_dim; ++i) {
    z[dst_root + i] = multi_level_z[padded_id * block_dim + i];
  }
  // Coarse space.
  for (int lv = 0; lv < coarse_level_num; ++lv) {
    int cco_id = coarse_space_map[real_id * MAS_MAX_COARSE_LEVEL + lv];
    int src_root = (level_offsets[lv + 1] + cco_id) * block_dim;
    for (int i = 0; i < block_dim; ++i) {
      z[dst_root + i] += multi_level_z[src_root + i];
    }
  }
}

__global__ void pad_zero_diagonal(float *mats, int mat_num, int mat_dim,
                                  int mat_storage_size) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  int total_diag = mat_num * mat_dim;
  if (tid >= total_diag) {
    return;
  }

  int mat_id = tid / mat_dim;
  int row = tid % mat_dim;
  float *mat = mats + mat_id * mat_storage_size;
  float *diag = mat + index_upper_mat(mat_dim, row, row);
  if (*diag == 0.0f) {
    *diag = 1.0f;
  }
}

/// @brief Invert a packed symmetric matrix in-place using symmetric
/// Gauss-Jordan sweeps.
template <int N>
__global__ void batched_invert_upper(float *d_matrices, bool *success) {
  int mat_idx = blockIdx.x;
  constexpr int STORAGE = N * (N + 1) / 2;
  float *d_A = d_matrices + mat_idx * STORAGE;

  __shared__ float s_A[STORAGE];
  __shared__ float s_col[N];
  __shared__ float s_pivot;
  int tx = threadIdx.x;

  for (int i = tx; i < STORAGE; i += N) {
    s_A[i] = d_A[i];
  }
  __syncthreads();

  for (int pivot = 0; pivot < N; ++pivot) {
    if (tx == 0) {
      s_pivot = s_A[index_upper_mat(N, pivot, pivot)];
      if (!ctd::isfinite(s_pivot) || s_pivot == 0.0f) {
        *success = false;
      }
    }
    __syncthreads();

    if (tx == pivot) {
      s_col[tx] = 0.0f;
    } else {
      int row = ctd::min(tx, pivot);
      int col = ctd::max(tx, pivot);
      s_col[tx] = s_A[index_upper_mat(N, row, col)];
    }
    __syncthreads();

    // All threads sweep the same column index in lockstep so that
    // s_A[index_upper_mat(N, tx, col)] maps each (tx, col) pair to a
    // different shared-memory bank (the row-stride in packed format is
    // coprime to 32).  This eliminates the bank conflicts that occur when
    // each thread starts at col=tx.  Division is hoisted out of the loop.
    if (tx != pivot) {
      float scale = s_col[tx] / s_pivot;
      for (int col = 0; col < N; ++col) {
        if (col < tx || col == pivot) {
          continue;
        }

        float updated = s_A[index_upper_mat(N, tx, col)] - scale * s_col[col];
        if (!ctd::isfinite(updated)) {
          *success = false;
        }
        s_A[index_upper_mat(N, tx, col)] = updated;
      }
    }
    __syncthreads();

    if (tx == pivot) {
      float updated = -1.0f / s_pivot;
      if (!ctd::isfinite(updated)) {
        *success = false;
      } else {
        s_A[index_upper_mat(N, pivot, pivot)] = updated;
      }
    } else {
      float updated = s_col[tx] / s_pivot;
      if (!ctd::isfinite(updated)) {
        *success = false;
      } else {
        int row = ctd::min(tx, pivot);
        int col = ctd::max(tx, pivot);
        s_A[index_upper_mat(N, row, col)] = updated;
      }
    }
    __syncthreads();
  }

  for (int i = tx; i < STORAGE; i += N) {
    float output = -s_A[i];
    if (!ctd::isfinite(output)) {
      *success = false;
    }
    d_A[i] = output;
  }
}

void invert_packed_matrices(float *mats, int mat_num, int block_dim,
                            CudaRuntime rt) {
  if (mat_num == 0) {
    return;
  }

  auto success = alloc<bool>(rt, 1, true);

  if (block_dim == 1) {
    batched_invert_upper<32>
        <<<mat_num, 32, 0, rt.stream.get()>>>(mats, success.data());
  } else if (block_dim == 2) {
    batched_invert_upper<64>
        <<<mat_num, 64, 0, rt.stream.get()>>>(mats, success.data());
  } else if (block_dim == 3) {
    batched_invert_upper<96>
        <<<mat_num, 96, 0, rt.stream.get()>>>(mats, success.data());
  } else {
    SILK_UNREACHABLE();
  }

  bool h_success = scalar_load(success.data(), rt);
  // Should not happen.
  if (!h_success) {
    throw std::runtime_error("MAS packed inverse failed.");
  }
}

/// @brief Build symmetric upper triangular coarse space matrices.
CoarseMatrices build_sym_coarse_matrices(const CoarseSpace &cs,
                                         DynamicBSRView mat_in,
                                         ctd::span<const int> real_to_padded,
                                         int padded_node_num, CudaRuntime rt) {
  BSRView mat = mat_in.mat;
  CoarseMatrices out;
  out.mat_dim = BANK_SIZE * mat.block_dim;
  out.mat_storage_size = out.mat_dim * (out.mat_dim + 1) / 2;
  out.total_matrix_num = 0;
  out.matrix_offsets[0] = 0;
  out.matrix_nums[0] = padded_node_num / BANK_SIZE;
  out.total_matrix_num += out.matrix_nums[0];
  for (int i = 0; i < cs.level_num; ++i) {
    out.matrix_offsets[i + 1] = out.total_matrix_num;
    out.matrix_nums[i + 1] = div_round_up(cs.cco_nums[i], BANK_SIZE);
    out.total_matrix_num += out.matrix_nums[i + 1];
  }

  out.data =
      alloc<float>(rt, out.total_matrix_num * out.mat_storage_size, 0.0f);
  CoarseMatricesRef view{out};

  // Gather.
  int grid_num = div_round_up(mat.dim, 128);
  fill_coarse_matrices<<<grid_num, 128, 0, rt.stream.get()>>>(
      mat_in, view, *cs.map, real_to_padded, cs.level_num);

  // Pad diagonal.
  int total_diag = out.total_matrix_num * out.mat_dim;
  grid_num = div_round_up(total_diag, 128);
  pad_zero_diagonal<<<grid_num, 128, 0, rt.stream.get()>>>(
      out.data->data(), out.total_matrix_num, out.mat_dim,
      out.mat_storage_size);

  // Compute inverse.
  invert_packed_matrices(out.data->data(), out.total_matrix_num, mat.block_dim,
                         rt);
  return out;
}
}  // namespace

void MASPreconditioner::factorize(DynamicBSRView A,
                                  ctd::span<const int> part_offsets,
                                  CudaRuntime rt) {
  assert(part_offsets.size() >= 2);
  assert(A.mat.block_dim >= 1 && A.mat.block_dim <= 3);
#ifndef NDEBUG
  int last_part_offset =
      scalar_load(part_offsets.data() + part_offsets.size() - 1, rt);
  assert(last_part_offset == A.mat.dim);
#endif
  assert(A.diag.size() == A.mat.dim * A.mat.block_dim);

  block_dim_ = A.mat.block_dim;

  auto total_begin = clock::now();
  auto phase_begin = clock::now();

  // Build padded topology.
  padded_topology_ = build_padded_topology(A.mat, part_offsets, rt);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_build_padded_topology] [{:.6f}]",
               elapsed(phase_begin));

  // Build coarse space hierarchy.
  phase_begin = clock::now();
  coarse_space_ =
      build_coarse_space(*(padded_topology_.rows), *(padded_topology_.cols),
                         *(padded_topology_.real_to_padded),
                         *(padded_topology_.padded_to_real), rt);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_build_coarse_space] [{:.6f}]", elapsed(phase_begin));

  // Build coarse space matrices by 1. gather coarse space hessian from fine
  // nodes 2. invert
  phase_begin = clock::now();
  coarse_matrices_ = build_sym_coarse_matrices(
      coarse_space_, A, *(padded_topology_.real_to_padded),
      padded_topology_.padded_node_num, rt);
  rt.stream.sync();
  SPDLOG_TRACE("[factorize_build_coarse_matrices] [{:.6f}]",
               elapsed(phase_begin));

  // Allocate memory for coarse space residual (r) and preconditioned residual
  // (z).
  phase_begin = clock::now();
  coarse_vectors_.level_offsets[0] = 0;
  coarse_vectors_.level_nums[0] = padded_topology_.padded_node_num;
  coarse_vectors_.total_level_nodes = padded_topology_.padded_node_num;
  for (int i = 0; i < coarse_space_.level_num; ++i) {
    coarse_vectors_.level_offsets[i + 1] = coarse_vectors_.total_level_nodes;
    coarse_vectors_.level_nums[i + 1] =
        coarse_matrices_.matrix_nums[i + 1] * BANK_SIZE;
    coarse_vectors_.total_level_nodes += coarse_vectors_.level_nums[i + 1];
  }
  int total_level_scalars = coarse_vectors_.total_level_nodes * block_dim_;
  coarse_vectors_.multi_level_r = alloc<float>(rt, total_level_scalars, 0.0f);
  coarse_vectors_.multi_level_z = alloc<float>(rt, total_level_scalars, 0.0f);
  rt.stream.sync();

  SPDLOG_TRACE("[factorize_allocate_coarse_vectors] [{:.6f}]",
               elapsed(phase_begin));
  SPDLOG_TRACE("[factorize_total] [{:.6f}]", elapsed(total_begin));
}

void MASPreconditioner::apply(ctd::span<const float> r, ctd::span<float> z,
                              CudaRuntime rt) {
  assert(r.size() == z.size());

  cu::fill_bytes(rt.stream, *(coarse_vectors_.multi_level_r), 0);
  cu::fill_bytes(rt.stream, *(coarse_vectors_.multi_level_z), 0);

  int gather_r_grid_num = div_round_up(padded_topology_.padded_node_num, 128);
  if (block_dim_ == 1) {
    gather_multi_level_r<1><<<gather_r_grid_num, 128, 0, rt.stream.get()>>>(
        r, *(coarse_vectors_.multi_level_r), *(padded_topology_.padded_to_real),
        *(coarse_space_.map), coarse_vectors_.level_offsets,
        coarse_space_.level_num);
  } else if (block_dim_ == 2) {
    gather_multi_level_r<2><<<gather_r_grid_num, 128, 0, rt.stream.get()>>>(
        r, *(coarse_vectors_.multi_level_r), *(padded_topology_.padded_to_real),
        *(coarse_space_.map), coarse_vectors_.level_offsets,
        coarse_space_.level_num);
  } else {
    gather_multi_level_r<3><<<gather_r_grid_num, 128, 0, rt.stream.get()>>>(
        r, *(coarse_vectors_.multi_level_r), *(padded_topology_.padded_to_real),
        *(coarse_space_.map), coarse_vectors_.level_offsets,
        coarse_space_.level_num);
  }

  apply_inverse(coarse_matrices_, *(coarse_vectors_.multi_level_r),
                *(coarse_vectors_.multi_level_z), block_dim_, rt);

  int gather_z_grid_num = div_round_up(padded_topology_.node_num, 128);
  gather_multi_level_z<<<gather_z_grid_num, 128, 0, rt.stream.get()>>>(
      *(coarse_vectors_.multi_level_z), z, *(padded_topology_.real_to_padded),
      *(coarse_space_.map), coarse_vectors_.level_offsets, block_dim_,
      coarse_space_.level_num);
}

}  // namespace silk::cuda
