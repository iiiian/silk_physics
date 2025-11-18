#include <cuda_runtime.h>

#include <cmath>
#include <cub/cub.cuh>
#include <iostream>

#include "backend/cuda/csr_matrix.hpp"
#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/device_vector.hpp"
#include "backend/cuda/solver/a_jacobi_solver.hpp"

namespace silk::cuda {

__global__ void compute_x_kernel(int n, CSRMatrixView d_R, const float* d_D,
                                 const float* d_rhs, const float* d_x_in,
                                 float* d_x_out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  if (tid < n) {
    int row_start = d_R.d_row_ptr[tid];
    int row_end = d_R.d_row_ptr[tid + 1];

    float accu = 0.0f;
    for (int i = row_start; i < row_end; ++i) {
      int col = d_R.d_col_idx[i];
      float val = d_R.d_values[i];
      accu += val * d_x_in[col];
    }

    d_x_out[tid] = (d_rhs[tid] - accu) / d_D[tid];
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

bool a_jacobi(int n, int max_iter, float abs_tol, float rel_tol,
              const CSRMatrix& d_R, const float* d_D, const float* d_rhs,
              float* d_x) {
  DVector<float> d_residual2(1);
  DVector<float> d_x_buffer(static_cast<size_t>(n));
  float h_residual2 = 0.0f;

  float* d_x_prev = d_x;
  float* d_x_next = d_x_buffer;

  float prev_residual = std::numeric_limits<float>::infinity();
  float residual;
  for (int iter = 0; iter < max_iter; ++iter) {
    int min_grid_size;
    int block_size;
    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                       compute_x_kernel, 0, 0);
    int grid_size = (n + block_size - 1) / block_size;
    compute_x_kernel<<<grid_size, block_size>>>(n, d_R.get_view(), d_D, d_rhs,
                                                d_x_prev, d_x_next);
    CHECK_CUDA(cudaMemset(d_residual2, 0, sizeof(float)));
    CHECK_CUDA(cudaDeviceSynchronize());
    CHECK_CUDA(cudaGetLastError());

    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                       compute_residial2_kernel, 0, 0);
    grid_size = (n + 256 - 1) / 256;
    compute_residial2_kernel<<<grid_size, 256>>>(n, d_R.get_view(), d_D, d_rhs,
                                                 d_x_next, d_residual2);
    CHECK_CUDA(cudaDeviceSynchronize());
    CHECK_CUDA(cudaGetLastError());
    CHECK_CUDA(cudaMemcpy(&h_residual2, d_residual2, sizeof(float),
                          cudaMemcpyDeviceToHost));

    residual = std::sqrt(h_residual2);
    float rel_diff = (prev_residual - residual) / residual;
    if (residual > prev_residual) {
      std::cout << "Jacobi solver residual increase: iter " << iter
                << ", curr residual " << residual << ", abs tol " << abs_tol
                << ", rel residual diff " << rel_diff << "\n";
      // return false;
    }
    if (residual < abs_tol || (rel_diff < rel_tol)) {
      // Ensure the latest solution lives in the user buffer d_x.
      if (d_x_next != d_x) {
        CHECK_CUDA(cudaMemcpy(d_x, d_x_next, n * sizeof(float),
                              cudaMemcpyDeviceToDevice));
      }

      std::cout << "Jacobi solver terminate: iter " << iter
                << ", curr residual " << residual << ", abs tol " << abs_tol
                << ", rel residual diff " << rel_diff << "\n";
      return true;
    }

    prev_residual = residual;
    std::swap(d_x_prev, d_x_next);
  }
  // On failure, still copy the last iterate back if needed.
  if (d_x_prev != d_x) {
    CHECK_CUDA(
        cudaMemcpy(d_x, d_x_prev, n * sizeof(float), cudaMemcpyDeviceToDevice));
  }

  std::cout << "Jacobi solver fail: iter " << max_iter << ", curr residual "
            << residual << ", abs tol " << abs_tol << ", rel residual diff "
            << (prev_residual - residual) / residual << "\n";
  return false;
}

}  // namespace silk::cuda
