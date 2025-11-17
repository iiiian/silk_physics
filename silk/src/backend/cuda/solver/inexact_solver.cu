#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cuda_runtime.h>

//to use:
// step (1) 
//      Eigen::MatrixXf U;
//      bool ok = construct_U(H, r, U);
// step (2)
//      U->du and cpy all the other required para into device
//      inexact_solver_cuda(dH, dU, n, dX, dB, n, r, dXout);
// step (3)
//      Eigen::VectorXf x_new(n);
//       cudaMemcpy(x_new.data(), dXout, sizeof(float)*n, cudaMemcpyDeviceToHost);

/** Simple CUDA error guard (only used if the project has no own macro). */
#ifndef CUDA_CHECK
#define CUDA_CHECK(expr)                                       \
  do {                                                         \
    cudaError_t __err = (expr);                                \
    if (__err != cudaSuccess) {                                \
      fprintf(stderr, "[CUDA] %s failed: %s\n", #expr,         \
              cudaGetErrorString(__err));                      \
      std::abort();                                            \
    }                                                          \
  } while (0)
#endif

/** Upper bound for subspace dimension r (kept as originally used). */
#ifndef R_MAX
#define R_MAX 128
#endif

/** Device-side CSR container (kept as-is to avoid ABI changes). */
struct CsrDevice {
  int n;            // dimension
  int nnz;          // number of nonzeros
  const int* Ap;    // row pointer, length n+1
  const int* Aj;    // column index, length nnz
  const float* Ax;  // values, length nnz
};

namespace {  // ======================= internal helpers =======================

/**
 * @brief y = A * x (CSR SpMV)
 * @param n     number of rows
 * @param Ap    CSR row pointer, length n+1
 * @param Aj    CSR column indices, length nnz
 * @param Ax    CSR values, length nnz
 * @param x     dense input vector, length n
 * @param y     dense output vector, length n
 */
__global__ void csr_spmv_kernel(int n,
                                const int* Ap,
                                const int* Aj,
                                const float* Ax,
                                const float* x,
                                float* y) {
  int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= n) {
    return;
  }
  float sum = 0.f;
  const int start = Ap[row];
  const int end = Ap[row + 1];
  for (int k = start; k < end; ++k) {
    sum += Ax[k] * x[Aj[k]];
  }
  y[row] = sum;
}

/**
 * @brief y = x (vector copy)
 */
__global__ void vec_copy_kernel(int n, const float* x, float* y) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    y[i] = x[i];
  }
}

/**
 * @brief y = a * x + b * y (in-place axpby)
 */
__global__ void axpby_inplace_kernel(int n,
                                     float a,
                                     const float* x,
                                     float b,
                                     float* y) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    y[i] = a * x[i] + b * y[i];
  }
}

/**
 * @brief Compute V = H * U column-by-column on device.
 *        U: (n x r, col-major, ld_u = n), V: (n x r, col-major, ld_v = n).
 */
inline void compute_hu_on_gpu(const CsrDevice& H,
                              const float* d_u,
                              int ld_u,
                              float* d_v,
                              int ld_v,
                              int r) {
  const int threads = 256;
  const int blocks = (H.n + threads - 1) / threads;
  for (int j = 0; j < r; ++j) {
    const float* col_u = d_u + j * ld_u;
    float* col_v = d_v + j * ld_v;
    csr_spmv_kernel<<<blocks, threads>>>(H.n, H.Ap, H.Aj, H.Ax, col_u, col_v);
    CUDA_CHECK(cudaGetLastError());
  }
}

/**
 * @brief A = U^T * V, where U and V are (n x r) col-major (ld = n),
 *        and A is a compact (r x r) row-major buffer on device.
 */
__global__ void gram_utv_kernel(int n,
                                const float* U,
                                int ld_u,
                                const float* V,
                                int ld_v,
                                int r,
                                float* A) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;  // column in A
  int i = blockIdx.y * blockDim.y + threadIdx.y;  // row in A
  if (i >= r || j >= r) {
    return;
  }
  float sum = 0.f;
  for (int k = 0; k < n; ++k) {
    sum += U[k + i * ld_u] * V[k + j * ld_v];
  }
  A[i * r + j] = sum;
}

/**
 * @brief rhs = U^T * r_vec, where U is (n x r) col-major (ld = n).
 */
__global__ void ut_vec_kernel(int n,
                              const float* U,
                              int ld_u,
                              int r,
                              const float* r_vec,
                              float* rhs) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;  // 0..r-1
  if (i >= r) {
    return;
  }
  float sum = 0.f;
  for (int k = 0; k < n; ++k) {
    sum += U[k + i * ld_u] * r_vec[k];
  }
  rhs[i] = sum;
}

/**
 * @brief In-place Cholesky factorization (lower) for small r x r SPD matrix A.
 * @return true if success, false if matrix is not SPD.
 */
__device__ bool chol_decomp_inplace(float* A, int r) {
  for (int j = 0; j < r; ++j) {
    float sum = 0.f;
    for (int k = 0; k < j; ++k) {
      sum += A[j * r + k] * A[j * r + k];
    }
    float d = A[j * r + j] - sum;
    if (d <= 0.f) {
      return false;  // not SPD
    }
    const float l_jj = sqrtf(d);
    A[j * r + j] = l_jj;

    for (int i = j + 1; i < r; ++i) {
      float s = 0.f;
      for (int k = 0; k < j; ++k) {
        s += A[i * r + k] * A[j * r + k];
      }
      A[i * r + j] = (A[i * r + j] - s) / l_jj;
    }
    // clear upper for clarity
    for (int k = j + 1; k < r; ++k) {
      A[j * r + k] = 0.f;
    }
  }
  return true;
}

/**
 * @brief Solve L L^T x = b using forward/back substitution.
 *        Requires a small shared buffer for y.
 */
__device__ void chol_solve_llt(const float* L,
                               const float* b,
                               float* x,
                               int r) {
  extern __shared__ float s[];
  float* y = s;

  // forward: L y = b
  for (int i = 0; i < r; ++i) {
    float sum = 0.f;
    for (int k = 0; k < i; ++k) {
      sum += L[i * r + k] * y[k];
    }
    y[i] = (b[i] - sum) / L[i * r + i];
  }
  // backward: L^T x = y
  for (int i = r - 1; i >= 0; --i) {
    float sum = 0.f;
    for (int k = i + 1; k < r; ++k) {
      sum += L[k * r + i] * x[k];
    }
    x[i] = (y[i] - sum) / L[i * r + i];
  }
}

/**
 * @brief Device-side Cholesky solve for small r x r system.
 *        On exit A stores L (lower), q is the solution of (A_original) q = rhs.
 */
__global__ void small_chol_solve_kernel(float* A,
                                        float* rhs,
                                        float* q,
                                        int r) {
  if (threadIdx.x == 0) {
    bool ok = chol_decomp_inplace(A, r);
    if (!ok) {
      // light diagonal damping to stabilize
      for (int i = 0; i < r; ++i) {
        A[i * r + i] += 1e-6f;
      }
      (void)chol_decomp_inplace(A, r);
    }
  }
  __syncthreads();

  extern __shared__ float s_buf[];
  // zero-initialize q in parallel
  for (int i = threadIdx.x; i < r; i += blockDim.x) {
    q[i] = 0.f;
  }
  __syncthreads();

  if (threadIdx.x == 0) {
    chol_solve_llt(A, rhs, q, r);
  }
}

/**
 * @brief x_out = X + U q  (col-major U with ld = n).
 */
__global__ void combine_update_kernel(int n,
                                      const float* U,
                                      int ld_u,
                                      int r,
                                      const float* q,
                                      const float* X,
                                      float* x_out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  float dx = 0.f;
  for (int j = 0; j < r; ++j) {
    dx += U[i + j * ld_u] * q[j];
  }
  x_out[i] = X[i] + dx;
}

}  // namespace

// =============================== public entry ===============================

/**
 * @brief Single inexact subspace correction on GPU.
 * @param H        SPD system matrix in device-CSR format.
 * @param d_U      Subspace basis U on device, shape (n x r), col-major.
 * @param ldU      Leading dimension of U (usually n).
 * @param d_X      Current iterate x (device, length n).
 * @param d_b      Right-hand side b (device, length n).
 * @param n        Problem size.
 * @param r        Subspace dimension.
 * @param d_x_out  Output vector x_new = X + U * q (device, length n).
 */
extern "C" void inexact_solver_cuda(const CsrDevice& H,
                                    const float* d_U,
                                    int ldU,
                                    const float* d_X,
                                    const float* d_b,
                                    int n,
                                    int r,
                                    float* d_x_out) {
  assert(n == H.n);
  assert(r > 0 && r <= R_MAX);

  // working buffers
  float *d_hx = nullptr, *d_r = nullptr;
  float *d_v = nullptr, *d_a = nullptr;
  float *d_rhs = nullptr, *d_q = nullptr;

  CUDA_CHECK(cudaMalloc(&d_hx, n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_r, n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_v, n * r * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_a, r * r * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_rhs, r * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_q, r * sizeof(float)));

  // 1) hX = H * X
  {
    const int threads = 256;
    const int blocks = (n + threads - 1) / threads;
    csr_spmv_kernel<<<blocks, threads>>>(n, H.Ap, H.Aj, H.Ax, d_X, d_hx);
    CUDA_CHECK(cudaGetLastError());
  }

  // 2) r = b - hX
  {
    const int threads = 256;
    const int blocks = (n + threads - 1) / threads;
    vec_copy_kernel<<<blocks, threads>>>(n, d_b, d_r);
    CUDA_CHECK(cudaGetLastError());
    axpby_inplace_kernel<<<blocks, threads>>>(n, 1.f, d_r, -1.f, d_hx);  // hX = r - hX
    CUDA_CHECK(cudaGetLastError());
    vec_copy_kernel<<<blocks, threads>>>(n, d_hx, d_r);                  // r  = b - Hx
    CUDA_CHECK(cudaGetLastError());
  }

  // 3) V = H * U
  compute_hu_on_gpu(H, d_U, ldU, d_v, n, r);

  // 4) A = U^T * V
  {
    dim3 t(16, 16);
    dim3 b((r + t.x - 1) / t.x, (r + t.y - 1) / t.y);
    gram_utv_kernel<<<b, t>>>(n, d_U, ldU, d_v, n, r, d_a);
    CUDA_CHECK(cudaGetLastError());
  }

  // 5) rhs = U^T * r
  {
    const int threads = 256;
    const int blocks = (r + threads - 1) / threads;
    ut_vec_kernel<<<blocks, threads>>>(n, d_U, ldU, r, d_r, d_rhs);
    CUDA_CHECK(cudaGetLastError());
  }

  // 6) solve A q = rhs (device Cholesky)
  {
    const int threads = 64;
    const size_t smem = sizeof(float) * r;  // shared buffer for y
    small_chol_solve_kernel<<<1, threads, smem>>>(d_a, d_rhs, d_q, r);
    CUDA_CHECK(cudaGetLastError());
  }

  // 7) x = X + U q
  {
    const int threads = 256;
    const int blocks = (n + threads - 1) / threads;
    combine_update_kernel<<<blocks, threads>>>(n, d_U, ldU, r, d_q, d_X, d_x_out);
    CUDA_CHECK(cudaGetLastError());
  }

  cudaFree(d_hx);
  cudaFree(d_r);
  cudaFree(d_v);
  cudaFree(d_a);
  cudaFree(d_rhs);
  cudaFree(d_q);
}
