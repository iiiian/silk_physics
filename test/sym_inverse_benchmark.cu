#include <cuda_runtime.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cuda/std/algorithm>
#include <cuda/std/cmath>
#include <cuda/std/utility>
#include <random>
#include <vector>

namespace {

namespace ctd = cuda::std;

__host__ __device__ int index_upper_mat(int dim, int i, int j) {
  if (i > j) {
    ctd::swap(i, j);
  }
  return i * dim - (i * (i + 1) / 2) + j;
}

// Same as the kernel in mas_preconditioner.cu
template <int N>
__global__ void batched_invert_upper_original(float *d_matrices,
                                              bool *success) {
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

    if (tx != pivot) {
      float a_ik = s_col[tx];
      for (int col = tx; col < N; ++col) {
        if (col == pivot) {
          continue;
        }

        float updated =
            s_A[index_upper_mat(N, tx, col)] - a_ik * s_col[col] / s_pivot;
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

// Optimized: column-by-column sweep to eliminate bank conflicts + hoisted
// division
template <int N>
__global__ void batched_invert_upper_optimized(float *d_matrices,
                                               bool *success) {
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

void generate_random_spd_upper(float *host_mat, int N, std::mt19937 &rng) {
  std::uniform_real_distribution<float> off_diag_dist(-0.01f, 0.01f);

  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      host_mat[index_upper_mat(N, i, j)] = off_diag_dist(rng);
    }
    float diag_sum = 0.0f;
    for (int j = 0; j < N; ++j) {
      if (i == j) continue;
      int row = std::min(i, j);
      int col = std::max(i, j);
      diag_sum += std::abs(host_mat[index_upper_mat(N, row, col)]);
    }
    host_mat[index_upper_mat(N, i, i)] = diag_sum + 1.0f;
  }
}

void unpack_upper_to_full(const float *upper, Eigen::MatrixXf &full, int N) {
  full.setZero();
  for (int i = 0; i < N; ++i) {
    for (int j = i; j < N; ++j) {
      float val = upper[index_upper_mat(N, i, j)];
      full(i, j) = val;
      full(j, i) = val;
    }
  }
}

void pack_full_to_upper(const Eigen::MatrixXf &full, float *upper, int N) {
  for (int i = 0; i < N; ++i) {
    for (int j = i; j < N; ++j) {
      upper[index_upper_mat(N, i, j)] = full(i, j);
    }
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  int num_matrices = 3000;
  if (argc > 1) {
    num_matrices = std::atoi(argv[1]);
  }

  constexpr int N = 96;
  constexpr int STORAGE = N * (N + 1) / 2;

  printf("Benchmark: batched_invert_upper (N=%d, matrices=%d)\n", N,
         num_matrices);
  printf("Shared mem per block: %zu bytes\n",
         STORAGE * sizeof(float) + N * sizeof(float) + sizeof(float));

  std::mt19937 rng(42);
  std::vector<float> h_matrices(num_matrices * STORAGE);
  for (int m = 0; m < num_matrices; ++m) {
    generate_random_spd_upper(h_matrices.data() + m * STORAGE, N, rng);
  }

  float *d_buf;
  bool *d_success;
  cudaMalloc(&d_buf, num_matrices * STORAGE * sizeof(float));
  cudaMalloc(&d_success, sizeof(bool));

  constexpr int WARMUP_ITERS = 5;
  constexpr int BENCH_ITERS = 20;

  auto run_benchmark = [&](const char *name, auto kernel_fn) -> float {
    cudaMemcpy(d_buf, h_matrices.data(), num_matrices * STORAGE * sizeof(float),
               cudaMemcpyHostToDevice);
    bool h_success_init = true;
    cudaMemcpy(d_success, &h_success_init, sizeof(bool),
               cudaMemcpyHostToDevice);

    for (int i = 0; i < WARMUP_ITERS; ++i) {
      kernel_fn<<<num_matrices, N>>>(d_buf, d_success);
    }
    cudaDeviceSynchronize();

    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaEventRecord(start);
    for (int i = 0; i < BENCH_ITERS; ++i) {
      kernel_fn<<<num_matrices, N>>>(d_buf, d_success);
    }
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    float ms_total = 0;
    cudaEventElapsedTime(&ms_total, start, stop);
    float ms_per_kernel = ms_total / BENCH_ITERS;

    bool success = true;
    cudaMemcpy(&success, d_success, sizeof(bool), cudaMemcpyDeviceToHost);

    printf("  %s: %.3f ms/kernel (success=%s)\n", name, ms_per_kernel,
           success ? "true" : "false");

    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    return ms_per_kernel;
  };

  float ms_orig = run_benchmark("Original", batched_invert_upper_original<N>);
  float ms_opt = run_benchmark("Optimized", batched_invert_upper_optimized<N>);

  printf("Speedup: %.2fx\n", ms_orig / ms_opt);

  // Correctness: compare each kernel output against Eigen inverse
  Eigen::MatrixXf mat_full(N, N);
  Eigen::MatrixXf inv_full(N, N);
  std::vector<float> h_result(num_matrices * STORAGE);
  int check_count = std::min(num_matrices, 100);

  auto check_kernel = [&](const char *name, auto kernel_fn) {
    cudaMemcpy(d_buf, h_matrices.data(), num_matrices * STORAGE * sizeof(float),
               cudaMemcpyHostToDevice);
    bool h_success_init = true;
    cudaMemcpy(d_success, &h_success_init, sizeof(bool),
               cudaMemcpyHostToDevice);
    kernel_fn<<<num_matrices, N>>>(d_buf, d_success);
    cudaMemcpy(h_result.data(), d_buf, num_matrices * STORAGE * sizeof(float),
               cudaMemcpyDeviceToHost);

    double max_rel_err = 0;
    double avg_rel_err = 0;
    for (int m = 0; m < check_count; ++m) {
      unpack_upper_to_full(h_matrices.data() + m * STORAGE, mat_full, N);
      Eigen::MatrixXf eigen_inv = mat_full.inverse();
      for (int i = 0; i < N; ++i) {
        for (int j = i; j < N; ++j) {
          float device_val = h_result[index_upper_mat(N, i, j) + m * STORAGE];
          float eigen_val = eigen_inv(i, j);
          double denom = std::max(std::abs((double)eigen_val), 1e-3);
          double rel_err =
              std::abs((double)device_val - (double)eigen_val) / denom;
          max_rel_err = std::max(max_rel_err, rel_err);
          avg_rel_err += rel_err;
        }
      }
    }
    avg_rel_err /= (double)check_count * N * (N + 1) / 2;
    printf("  %s vs Eigen: max_rel_err=%.6e, avg_rel_err=%.6e\n", name,
           max_rel_err, avg_rel_err);
  };

  check_kernel("Original", batched_invert_upper_original<N>);
  check_kernel("Optimized", batched_invert_upper_optimized<N>);

  cudaFree(d_buf);
  cudaFree(d_success);

  return 0;
}
