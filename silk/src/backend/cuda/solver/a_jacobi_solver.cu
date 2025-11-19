#include <cuda_runtime.h>

#include <cmath>
#include <cub/cub.cuh>
#include <iostream>

#include "backend/cuda/csr_matrix.hpp"
#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/device_vector.hpp"
#include "backend/cuda/solver/a_jacobi_solver.hpp"

namespace silk::cuda {

__global__ void compute_x_block_kernel(int n, CSRMatrixView d_R,
                                       const float* d_D, const float* d_rhs,
                                       const float* d_x_in, float* d_x_out,
                                       int inner_iters) {
  extern __shared__ float s_data[];
  float* s_curr = s_data;
  float* s_next = s_data + blockDim.x;

  int local_id = threadIdx.x;
  int global_row = blockIdx.x * blockDim.x + local_id;
  int block_row_start = blockIdx.x * blockDim.x;

  if (global_row < n) {
    s_curr[local_id] = d_x_in[global_row];
  }
  __syncthreads();

  for (int it = 0; it < inner_iters; ++it) {
    if (global_row < n) {
      int row_start = d_R.d_row_ptr[global_row];
      int row_end = d_R.d_row_ptr[global_row + 1];

      float accu = 0.0f;
      for (int i = row_start; i < row_end; ++i) {
        int col = d_R.d_col_idx[i];
        float val = d_R.d_values[i];
        float x_val;
        if (col >= block_row_start && col < block_row_start + blockDim.x) {
          int local_col = col - block_row_start;
          x_val = s_curr[local_col];
        } else {
          x_val = d_x_in[col];
        }
        accu += val * x_val;
      }

      s_next[local_id] = (d_rhs[global_row] - accu) / d_D[global_row];
    }

    __syncthreads();

    float* tmp = s_curr;
    s_curr = s_next;
    s_next = tmp;

    __syncthreads();
  }

  if (global_row < n) {
    d_x_out[global_row] = s_curr[local_id];
  }
}

__global__ void compute_residial2_kernel(int n, CSRMatrixView d_R,
                                         const float* d_D, const float* d_rhs,
                                         const float* d_x, float* d_residual2) {
  using BlockReduce = cub::BlockReduce<float, 256>;
  __shared__ typename BlockReduce::TempStorage temp_storage;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float diff2 = 0.0f;
  if (tid < n) {
    int row_start = d_R.d_row_ptr[tid];
    int row_end = d_R.d_row_ptr[tid + 1];

    float accu = 0.0f;
    for (int i = row_start; i < row_end; ++i) {
      int col = d_R.d_col_idx[i];
      float val = d_R.d_values[i];
      accu += val * d_x[col];
    }

    // Residual on (D + R) x = b.
    float Ax_i = d_D[tid] * d_x[tid] + accu;
    float diff = Ax_i - d_rhs[tid];
    diff2 = diff * diff;
  }

  float diff2_sum = BlockReduce(temp_storage).Sum(diff2);
  if (threadIdx.x == 0) {
    atomicAdd(d_residual2, diff2_sum);
  }
}

// Compute L_inf distance between two device vectors:
//   max_i |a_i - b_i|.
__global__ void linf_diff_kernel(int n, const float* d_a, const float* d_b,
                                 float* d_out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < n) {
    float diff = d_a[tid] - d_b[tid];
    d_out[tid] = fabsf(diff);
  }
}

float compute_Linf_dist(int n, const float* d_a, const float* d_b,
                        float* d_buffer) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     linf_diff_kernel, 0, 0);
  int grid_size = (n + block_size - 1) / block_size;
  linf_diff_kernel<<<grid_size, block_size>>>(n, d_a, d_b, d_buffer);
  cudaDeviceSynchronize();

  float* d_max = nullptr;
  CHECK_CUDA(cudaMalloc((void**)&d_max, sizeof(float)));
  // CUB DeviceReduce requires a two-phase API with temp storage.
  void* d_temp = nullptr;
  size_t temp_bytes = 0;
  cub::DeviceReduce::Max(nullptr, temp_bytes, d_buffer, d_max, n);
  CHECK_CUDA(cudaMalloc(&d_temp, temp_bytes));
  cub::DeviceReduce::Max(d_temp, temp_bytes, d_buffer, d_max, n);
  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  float h_max = 0.0f;
  CHECK_CUDA(cudaMemcpy(&h_max, d_max, sizeof(float), cudaMemcpyDeviceToHost));
  CHECK_CUDA(cudaFree(d_temp));
  CHECK_CUDA(cudaFree(d_max));

  return h_max;
}

bool a_jacobi(int n, int max_iter, float abs_tol, float rel_tol,
              const CSRMatrix& d_R, const float* d_D, const float* d_rhs,
              float* d_x) {
  DVector<float> d_x_buffer(static_cast<size_t>(n));
  float* d_x_prev = d_x;
  float* d_x_next = d_x_buffer;
  float Linf_dist0;
  float Linf_dist;

  constexpr int ACCUM_RUN = 2;
  constexpr int ACCUM_ITER = 16;

  // int min_grid_size;
  // int block_size;
  // cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
  //                                    compute_x_block_kernel, 0, 0);
  int block_size = 128;
  int grid_size = (n + block_size - 1) / block_size;
  size_t shmem_bytes = 2 * static_cast<size_t>(block_size) * sizeof(float);

  for (int iter = 0; iter < max_iter; iter += ACCUM_ITER) {
    for (int j = 0; j < ACCUM_ITER; ++j) {
      compute_x_block_kernel<<<grid_size, block_size, shmem_bytes>>>(
          n, d_R.get_view(), d_D, d_rhs, d_x_prev, d_x_next, ACCUM_RUN);

      CHECK_CUDA(cudaDeviceSynchronize());
      CHECK_CUDA(cudaGetLastError());

      // After the kernel, d_x_next holds the newest iterate and d_x_prev holds
      // the previous one. Swap so that d_x_prev always points to the latest
      // solution.
      std::swap(d_x_prev, d_x_next);
    }

    // d_x_prev: newest iterate, d_x_next: previous iterate (used as scratch).
    Linf_dist = compute_Linf_dist(n, d_x_prev, d_x_next, d_x_next);
    if (iter == 0) {
      Linf_dist0 = Linf_dist;
    }

    if (Linf_dist < abs_tol || (Linf_dist < rel_tol * Linf_dist0)) {
      // Ensure the latest solution lives in the user buffer d_x.
      if (d_x_prev != d_x) {
        CHECK_CUDA(cudaMemcpy(d_x, d_x_prev, n * sizeof(float),
                              cudaMemcpyDeviceToDevice));
      }

      // std::cout << "Jacobi solver terminate: iter " << iter << ", Linf dist "
      //           << Linf_dist << ", abs tol " << abs_tol << ", rel dist tol "
      //           << rel_tol * Linf_dist0 << "\n";
      return true;
    }
  }

  // On failure, still copy the last iterate back if needed.
  if (d_x_prev != d_x) {
    CHECK_CUDA(
        cudaMemcpy(d_x, d_x_prev, n * sizeof(float), cudaMemcpyDeviceToDevice));
  }

  // std::cout << "Jacobi solver terminate: iter " << max_iter << ", Linf dist "
  //           << Linf_dist << ", abs tol " << abs_tol << ", rel dist tol "
  //           << rel_tol * Linf_dist0 << "\n";
  return true;
}

}  // namespace silk::cuda
