#include <Eigen/SparseCore>
#include <cassert>
#include <cstdint>
#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/buffer>
#include <cuda/std/functional>
#include <cuda/std/span>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "backend/cuda/bsr_adjacency.cuh"

namespace silk::cuda {

namespace {

// Key bit layout:
// | block row idx (32) | block col idx (32) |

__both__ uint64_t pack_key(int row, int col) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(row)) << 32) |
         static_cast<uint32_t>(col);
}

__device__ int key_to_row(uint64_t key) {
  return static_cast<int>(static_cast<uint32_t>(key >> 32));
}

__device__ int key_to_col(uint64_t key) {
  return static_cast<int>(static_cast<uint32_t>(key));
}

__global__ void build_col_of_nnz(int col_num, ctd::span<const int> col_ptr,
                                 ctd::span<int> col_of_nnz) {
  int col = blockDim.x * blockIdx.x + threadIdx.x;
  if (col >= col_num) {
    return;
  }

  int begin = col_ptr[col];
  int end = col_ptr[col + 1];
  for (int k = begin; k < end; ++k) {
    col_of_nnz[k] = col;
  }
}

__global__ void build_directed_keys_vals_flags(int nnz, int block_dim,
                                               ctd::span<const int> col_of_nnz,
                                               ctd::span<const int> row_idx,
                                               ctd::span<const float> vals,
                                               ctd::span<uint64_t> keys,
                                               ctd::span<float> norm2,
                                               ctd::span<int> keep_flags) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= nnz) {
    return;
  }

  // Real node from input A. compute block ij then pack.
  int block_row = row_idx[tid] / block_dim;
  int block_col = col_of_nnz[tid] / block_dim;
  keys[tid] = pack_key(block_row, block_col);

  float val = vals[tid];
  norm2[tid] = val * val;
  keep_flags[tid] = (block_row != block_col) ? 1 : 0;
}

__global__ void apply_sqrt(ctd::span<float> vals) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= vals.size()) {
    return;
  }

  vals[tid] = sqrt(vals[tid]);
}

__global__ void extract_block_rows(ctd::span<const uint64_t> unique_keys,
                                   ctd::span<int> block_rows) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= unique_keys.size()) {
    return;
  }

  block_rows[tid] = key_to_row(unique_keys[tid]);
}

__global__ void extract_block_cols(ctd::span<const uint64_t> unique_keys,
                                   ctd::span<int> block_cols) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= unique_keys.size()) {
    return;
  }

  block_cols[tid] = key_to_col(unique_keys[tid]);
}

__global__ void build_pair_keys_indices(ctd::span<const uint64_t> directed_keys,
                                        ctd::span<uint64_t> pair_keys,
                                        ctd::span<int> orig_indices) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= directed_keys.size()) {
    return;
  }

  int row = key_to_row(directed_keys[tid]);
  int col = key_to_col(directed_keys[tid]);
  int u = row < col ? row : col;
  int v = row < col ? col : row;
  pair_keys[tid] = pack_key(u, v);
  orig_indices[tid] = tid;
}

__global__ void average_sym_weights(ctd::span<const int> pair_offsets,
                                    ctd::span<const int> sorted_orig_indices,
                                    ctd::span<const float> directed_weights,
                                    ctd::span<float> sym_weights) {
  int pair_id = blockDim.x * blockIdx.x + threadIdx.x;
  if (pair_id >= pair_offsets.size() - 1) {
    return;
  }

  int begin = pair_offsets[pair_id];
  int end = pair_offsets[pair_id + 1];
  if (begin >= end) {
    return;
  }

  float sum = 0.0f;
  for (int i = begin; i < end; ++i) {
    sum += directed_weights[sorted_orig_indices[i]];
  }
  float avg = sum / (end - begin);
  for (int i = begin; i < end; ++i) {
    sym_weights[sorted_orig_indices[i]] = avg;
  }
}

__global__ void quantize_weights(ctd::span<const float> weights_in,
                                 float min_weight, float max_weight,
                                 ctd::span<int64_t> weights_out) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid >= weights_in.size()) {
    return;
  }

  float range = max_weight - min_weight;
  if (range <= 0.0f) {
    weights_out[tid] = 1;
    return;
  }

  constexpr int MAX_QUANT = 1000000;
  float scaled = (weights_in[tid] - min_weight) / range * MAX_QUANT;
  int quant = static_cast<int>(scaled);
  if (quant < 1) {
    quant = 1;
  } else if (quant > MAX_QUANT) {
    quant = MAX_QUANT;
  }
  weights_out[tid] = static_cast<int64_t>(quant);
}

int build_adjacency_from_csc(const Eigen::SparseMatrix<float> &A_csc,
                             int block_dim, std::vector<int> &row_ptr,
                             std::vector<int> &cols,
                             std::vector<int64_t> &weights, CudaRuntime rt) {
  const int rows_num = A_csc.rows();
  const int cols_num = A_csc.cols();
  const int nnz = A_csc.nonZeros();
  const int dim_blocks = div_round_up(rows_num, block_dim);

  // Upload input CSC to device.
  Buf<int> d_col_ptr = alloc<int>(rt, cols_num + 1);
  Buf<int> d_row_idx = alloc<int>(rt, nnz);
  Buf<float> d_vals = alloc<float>(rt, nnz);
  cudaMemcpyAsync(d_col_ptr->data(), A_csc.outerIndexPtr(),
                  static_cast<size_t>(cols_num + 1) * sizeof(int),
                  cudaMemcpyHostToDevice, rt.stream.get());
  cudaMemcpyAsync(d_row_idx->data(), A_csc.innerIndexPtr(),
                  static_cast<size_t>(nnz) * sizeof(int),
                  cudaMemcpyHostToDevice, rt.stream.get());
  cudaMemcpyAsync(d_vals->data(), A_csc.valuePtr(),
                  static_cast<size_t>(nnz) * sizeof(float),
                  cudaMemcpyHostToDevice, rt.stream.get());

  // ---------------------------------------------------------------------------
  // Map each nnz index to input col.
  // ---------------------------------------------------------------------------

  Buf<int> col_of_nnz = alloc<int>(rt, nnz);
  build_col_of_nnz<<<div_round_up(cols_num, 128), 128, 0, rt.stream.get()>>>(
      cols_num, *d_col_ptr, *col_of_nnz);
  d_col_ptr->destroy();

  // ---------------------------------------------------------------------------
  // Convert each non-zero to a directed block key and weight.
  // ---------------------------------------------------------------------------

  Buf<uint64_t> all_keys = alloc<uint64_t>(rt, nnz);
  Buf<float> all_norm2 = alloc<float>(rt, nnz);
  Buf<int> keep_flags = alloc<int>(rt, nnz);
  build_directed_keys_vals_flags<<<div_round_up(nnz, 128), 128, 0,
                                   rt.stream.get()>>>(
      nnz, block_dim, *col_of_nnz, *d_row_idx, *d_vals, *all_keys, *all_norm2,
      *keep_flags);
  col_of_nnz->destroy();
  d_row_idx->destroy();

  Buf<char> cub_tmp;
  auto make_cub_tmp = [&cub_tmp, rt](size_t required_size) {
    if (!cub_tmp || cub_tmp->size() < required_size) {
      cub_tmp = alloc<char>(rt, required_size);
    }
    return cub_tmp->data();
  };

  // ---------------------------------------------------------------------------
  // Select off-diagonal directed block entries.
  // ---------------------------------------------------------------------------

  Buf<uint64_t> directed_keys = alloc<uint64_t>(rt, nnz);
  Buf<float> directed_norm2 = alloc<float>(rt, nnz);
  Buf<int> num_selected = alloc<int>(rt, 1);

  size_t select_tmp_size = 0;
  cub::DeviceSelect::Flagged(nullptr, select_tmp_size, all_keys->data(),
                             keep_flags->data(), directed_keys->data(),
                             num_selected->data(), nnz, rt.stream.get());
  cub::DeviceSelect::Flagged(make_cub_tmp(select_tmp_size), select_tmp_size,
                             all_keys->data(), keep_flags->data(),
                             directed_keys->data(), num_selected->data(), nnz,
                             rt.stream.get());
  int offdiag_nnz = scalar_load(num_selected->data(), rt);
  if (offdiag_nnz == 0) {
    row_ptr.assign(dim_blocks + 1, 0);
    cols.clear();
    weights.clear();
    return 0;
  }

  cub::DeviceSelect::Flagged(make_cub_tmp(select_tmp_size), select_tmp_size,
                             all_norm2->data(), keep_flags->data(),
                             directed_norm2->data(), num_selected->data(), nnz,
                             rt.stream.get());
  all_keys->destroy();
  all_norm2->destroy();
  keep_flags->destroy();
  num_selected->destroy();

  Buf<uint64_t> directed_keys_alt = alloc<uint64_t>(rt, offdiag_nnz);
  Buf<float> directed_norm2_alt = alloc<float>(rt, offdiag_nnz);
  cub::DoubleBuffer<uint64_t> d_keys(directed_keys->data(),
                                     directed_keys_alt->data());
  cub::DoubleBuffer<float> d_norm2(directed_norm2->data(),
                                   directed_norm2_alt->data());

  // ---------------------------------------------------------------------------
  // Radix sort by key. Result should be in row major order.
  // ---------------------------------------------------------------------------

  size_t sort_tmp_size = 0;
  cub::DeviceRadixSort::SortPairs(nullptr, sort_tmp_size, d_keys, d_norm2,
                                  offdiag_nnz, 0, 64, rt.stream.get());
  cub::DeviceRadixSort::SortPairs(make_cub_tmp(sort_tmp_size), sort_tmp_size,
                                  d_keys, d_norm2, offdiag_nnz, 0, 64,
                                  rt.stream.get());

  Buf<uint64_t> &sorted_key_buf = (d_keys.Current() == directed_keys->data())
                                      ? directed_keys
                                      : directed_keys_alt;
  Buf<float> &sorted_norm2_buf = (d_norm2.Current() == directed_norm2->data())
                                     ? directed_norm2
                                     : directed_norm2_alt;
  Buf<uint64_t> &stale_key_buf = (d_keys.Current() == directed_keys->data())
                                     ? directed_keys_alt
                                     : directed_keys;
  Buf<float> &stale_norm2_buf = (d_norm2.Current() == directed_norm2->data())
                                    ? directed_norm2_alt
                                    : directed_norm2;
  stale_key_buf->destroy();
  stale_norm2_buf->destroy();

  // ---------------------------------------------------------------------------
  // Reduce by key to compute one weighted edge per directed block pair.
  // ---------------------------------------------------------------------------

  Buf<uint64_t> unique_keys = alloc<uint64_t>(rt, offdiag_nnz);
  Buf<float> directed_weights = alloc<float>(rt, offdiag_nnz);
  Buf<int> edge_count_buf = alloc<int>(rt, 1);

  size_t reduce_tmp_size = 0;
  cub::DeviceReduce::ReduceByKey(
      nullptr, reduce_tmp_size, d_keys.Current(), unique_keys->data(),
      d_norm2.Current(), directed_weights->data(), edge_count_buf->data(),
      ctd::plus<>(), offdiag_nnz, rt.stream.get());
  cub::DeviceReduce::ReduceByKey(
      make_cub_tmp(reduce_tmp_size), reduce_tmp_size, d_keys.Current(),
      unique_keys->data(), d_norm2.Current(), directed_weights->data(),
      edge_count_buf->data(), ctd::plus<>(), offdiag_nnz, rt.stream.get());
  int edge_count = scalar_load(edge_count_buf->data(), rt);
  sorted_key_buf->destroy();
  sorted_norm2_buf->destroy();
  edge_count_buf->destroy();

  apply_sqrt<<<div_round_up(edge_count, 128), 128, 0, rt.stream.get()>>>(
      ctd::span<float>(directed_weights->data(), edge_count));

  // ---------------------------------------------------------------------------
  // Histogram + ExclusiveSum
  // This step count non-zero blocks per rows to compute row_ptr.
  // ---------------------------------------------------------------------------

  Buf<int> block_rows = alloc<int>(rt, edge_count);
  // Extract row index from packed key.
  extract_block_rows<<<div_round_up(edge_count, 128), 128, 0,
                       rt.stream.get()>>>(
      ctd::span<const uint64_t>(unique_keys->data(), edge_count), *block_rows);
  // Histogram count nnz block per row.
  Buf<int> hist = alloc<int>(rt, dim_blocks + 1, 0);
  size_t hist_tmp_size = 0;
  cub::DeviceHistogram::HistogramEven(
      nullptr, hist_tmp_size, block_rows->data(), hist->data(), dim_blocks + 1,
      0, dim_blocks, edge_count, rt.stream.get());
  cub::DeviceHistogram::HistogramEven(
      make_cub_tmp(hist_tmp_size), hist_tmp_size, block_rows->data(),
      hist->data(), dim_blocks + 1, 0, dim_blocks, edge_count, rt.stream.get());
  // Exclusive scan compute CSR row ptr.
  Buf<int> d_row_ptr = alloc<int>(rt, dim_blocks + 1);
  size_t scan_tmp_size = 0;
  cub::DeviceScan::ExclusiveSum(nullptr, scan_tmp_size, hist->data(),
                                d_row_ptr->data(), dim_blocks + 1,
                                rt.stream.get());
  cub::DeviceScan::ExclusiveSum(make_cub_tmp(scan_tmp_size), scan_tmp_size,
                                hist->data(), d_row_ptr->data(), dim_blocks + 1,
                                rt.stream.get());
  block_rows->destroy();
  hist->destroy();

  // ---------------------------------------------------------------------------
  // Fill cols and symmetrized weights.
  // ---------------------------------------------------------------------------

  // Fill cols.
  Buf<int> d_cols = alloc<int>(rt, edge_count);
  extract_block_cols<<<div_round_up(edge_count, 128), 128, 0,
                       rt.stream.get()>>>(
      ctd::span<const uint64_t>(unique_keys->data(), edge_count), *d_cols);

  // Build undirected pair keys so reciprocal directed weights can be averaged.
  Buf<uint64_t> pair_keys = alloc<uint64_t>(rt, edge_count);
  Buf<int> orig_indices = alloc<int>(rt, edge_count);
  build_pair_keys_indices<<<div_round_up(edge_count, 128), 128, 0,
                            rt.stream.get()>>>(
      ctd::span<const uint64_t>(unique_keys->data(), edge_count), *pair_keys,
      *orig_indices);

  Buf<uint64_t> pair_keys_alt = alloc<uint64_t>(rt, edge_count);
  Buf<int> orig_indices_alt = alloc<int>(rt, edge_count);
  cub::DoubleBuffer<uint64_t> d_pair_keys(pair_keys->data(),
                                          pair_keys_alt->data());
  cub::DoubleBuffer<int> d_orig_indices(orig_indices->data(),
                                        orig_indices_alt->data());

  // Radix sort by undirected pair key.
  size_t pair_sort_tmp_size = 0;
  cub::DeviceRadixSort::SortPairs(nullptr, pair_sort_tmp_size, d_pair_keys,
                                  d_orig_indices, edge_count, 0, 64,
                                  rt.stream.get());
  cub::DeviceRadixSort::SortPairs(
      make_cub_tmp(pair_sort_tmp_size), pair_sort_tmp_size, d_pair_keys,
      d_orig_indices, edge_count, 0, 64, rt.stream.get());

  Buf<uint64_t> &sorted_pair_keys =
      (d_pair_keys.Current() == pair_keys->data()) ? pair_keys : pair_keys_alt;
  Buf<int> &sorted_orig_indices =
      (d_orig_indices.Current() == orig_indices->data()) ? orig_indices
                                                         : orig_indices_alt;
  Buf<uint64_t> &stale_pair_keys =
      (d_pair_keys.Current() == pair_keys->data()) ? pair_keys_alt : pair_keys;
  Buf<int> &stale_orig_indices =
      (d_orig_indices.Current() == orig_indices->data()) ? orig_indices_alt
                                                         : orig_indices;
  stale_pair_keys->destroy();
  stale_orig_indices->destroy();

  Buf<uint64_t> unique_pair_keys = alloc<uint64_t>(rt, edge_count);
  Buf<int> pair_counts = alloc<int>(rt, edge_count + 1);
  Buf<int> pair_count_buf = alloc<int>(rt, 1);

  // ---------------------------------------------------------------------------
  // Run-length encode.
  // This step computes pair count + directed edge count for each pair.
  // ---------------------------------------------------------------------------

  size_t pair_rle_tmp_size = 0;
  cub::DeviceRunLengthEncode::Encode(
      nullptr, pair_rle_tmp_size, d_pair_keys.Current(),
      unique_pair_keys->data(), pair_counts->data(), pair_count_buf->data(),
      edge_count, rt.stream.get());
  cub::DeviceRunLengthEncode::Encode(
      make_cub_tmp(pair_rle_tmp_size), pair_rle_tmp_size, d_pair_keys.Current(),
      unique_pair_keys->data(), pair_counts->data(), pair_count_buf->data(),
      edge_count, rt.stream.get());
  int pair_count = scalar_load(pair_count_buf->data(), rt);
  sorted_pair_keys->destroy();
  unique_pair_keys->destroy();
  pair_count_buf->destroy();

  // Compute pair offsets for each undirected pair.
  Buf<int> pair_offsets = alloc<int>(rt, pair_count + 1);
  cudaMemsetAsync(pair_counts->data() + pair_count, 0, sizeof(int),
                  rt.stream.get());
  size_t pair_scan_tmp_size = 0;
  cub::DeviceScan::ExclusiveSum(nullptr, pair_scan_tmp_size,
                                pair_counts->data(), pair_offsets->data(),
                                pair_count + 1, rt.stream.get());
  cub::DeviceScan::ExclusiveSum(
      make_cub_tmp(pair_scan_tmp_size), pair_scan_tmp_size, pair_counts->data(),
      pair_offsets->data(), pair_count + 1, rt.stream.get());
  pair_counts->destroy();

  // Fill cols and weights.
  Buf<float> sym_weights = alloc<float>(rt, edge_count);
  average_sym_weights<<<div_round_up(pair_count, 128), 128, 0,
                        rt.stream.get()>>>(
      *pair_offsets, ctd::span<const int>(d_orig_indices.Current(), edge_count),
      ctd::span<const float>(directed_weights->data(), edge_count),
      *sym_weights);
  pair_offsets->destroy();
  sorted_orig_indices->destroy();
  directed_weights->destroy();
  unique_keys->destroy();

  // Quantize floating-point graph weights to positive integer edge weights.
  Buf<float> min_weight_buf = alloc<float>(rt, 1);
  Buf<float> max_weight_buf = alloc<float>(rt, 1);

  size_t min_tmp_size = 0;
  cub::DeviceReduce::Min(nullptr, min_tmp_size, sym_weights->data(),
                         min_weight_buf->data(), edge_count, rt.stream.get());
  cub::DeviceReduce::Min(make_cub_tmp(min_tmp_size), min_tmp_size,
                         sym_weights->data(), min_weight_buf->data(),
                         edge_count, rt.stream.get());

  size_t max_tmp_size = 0;
  cub::DeviceReduce::Max(nullptr, max_tmp_size, sym_weights->data(),
                         max_weight_buf->data(), edge_count, rt.stream.get());
  cub::DeviceReduce::Max(make_cub_tmp(max_tmp_size), max_tmp_size,
                         sym_weights->data(), max_weight_buf->data(),
                         edge_count, rt.stream.get());

  float min_weight = scalar_load(min_weight_buf->data(), rt);
  float max_weight = scalar_load(max_weight_buf->data(), rt);

  Buf<int64_t> d_weights = alloc<int64_t>(rt, edge_count);
  quantize_weights<<<div_round_up(edge_count, 128), 128, 0, rt.stream.get()>>>(
      *sym_weights, min_weight, max_weight, *d_weights);
  sym_weights->destroy();

  row_ptr.resize(dim_blocks + 1);
  cols.resize(edge_count);
  weights.resize(edge_count);
  cu::copy_bytes(rt.stream, *d_row_ptr, row_ptr);
  cu::copy_bytes(rt.stream, *d_cols, cols);
  cu::copy_bytes(rt.stream, *d_weights, weights);
  rt.stream.sync();

  return edge_count;
}
}  // namespace

BSRAdjacency::BSRAdjacency(const Eigen::SparseMatrix<float> &A, int block_dim,
                           CudaRuntime rt) {
  static_assert(std::is_same_v<Eigen::SparseMatrix<float>::StorageIndex, int>,
                "Only support int32 index type.");
  assert(A.cols() == A.rows() && A.cols() != 0 && A.rows() != 0 &&
         A.nonZeros() != 0);
  assert(A.cols() <= std::numeric_limits<int>::max() &&
         A.rows() <= std::numeric_limits<int>::max() &&
         A.nonZeros() <= std::numeric_limits<int>::max());
  assert(block_dim >= 1 && block_dim <= 3);

  // if A is not compressed. Copy and compress.
  const Eigen::SparseMatrix<float> *A_csc = &A;
  Eigen::SparseMatrix<float> compressed_A;
  if (!A.isCompressed()) {
    compressed_A = A;
    compressed_A.makeCompressed();
    A_csc = &compressed_A;
  }

  build_adjacency_from_csc(*A_csc, block_dim, row_ptr, cols, weights, rt);
}

}  // namespace silk::cuda
