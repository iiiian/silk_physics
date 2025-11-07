#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <cuda_runtime.h>

// **************** Blueprint ***************************
// In CPU, We let U = smallest_eigenvectors(H, r)  
// Then copy_to_device(H_csr, U, X, b)

// In GPU
// HX = spmv(H, X)
// r = b - HX
// V = [ spmv(H, U[:,j]) for j in 0..r-1 ]         
// A = U^T * V
// rhs = U^T * r
// q = chol_solve(A, rhs)                         
// x = X + U * q
//********************************************************

// 1）Eigen::MatrixXf U_host;
// 2）construct_U(H, r, U_host, /*max_subspace=*/-1, /*max_iter=*/1000, /*tol=*/1e-6)
// 3）float* d_U = nullptr;
//    CUDA_CHECK(cudaMalloc(&d_U, sizeof(float) * H.rows() * r));
//    CUDA_CHECK(cudaMemcpy(d_U, U_host.data(),sizeof(float) * H.rows() * r, cudaMemcpyHostToDevice));
//    int ldU = static_cast<int>(H.rows()); 
// 4）inexact_solver_cuda(dH, d_U, ldU, d_X, d_b, n, r, d_x_out);
















/**
 * @brief 外部暴露的 CUDA 入口（给集成方调用）
 * 在他们的 .cu/.cpp 顶部，只需复制下面这个“函数原型”即可：
 * extern "C" void inexact_solver_cuda(const struct CsrDevice& H,
 *                          const float* d_U, int ldU,
 *                          const float* d_X,
 *                          const float* d_b,
 *                          int n, int r,
 *                          float* d_x_out);
 */

// 设备侧 CSR 表示（按照你的工程约定）：
struct CsrDevice {
    int n;              // 维度
    int nnz;            // 非零个数
    const int*   Ap;    // 行指针，长度 n+1
    const int*   Aj;    // 列下标，长度 nnz
    const float* Ax;    // 数值，长度 nnz
};

// ---- 编译期检查与错误处理宏 ------------------------------
#define CUDA_CHECK(expr) do { \
    cudaError_t __err = (expr); \
    if (__err != cudaSuccess) { \
      fprintf(stderr, "[CUDA] %s failed at %s:%d: %s\n", \
              #expr, __FILE__, __LINE__, cudaGetErrorString(__err)); \
      std::exit(1); \
    } \
} while(0)

// 允许的最大子空间维度（可在 CMake add_compile_definitions(R_MAX=...) 覆盖）
#ifndef R_MAX
#define R_MAX 128
#endif

namespace { // =========================== 内部实现（匿名命名空间）===========================

/** y = A * x，其中 A 为 CSR（设备端），x/y 为密向量（设备端） */
__global__ void csr_spmv_kernel(int n, const int* Ap, const int* Aj, const float* Ax,
                                const float* x, float* y)
{
    int row = blockIdx.x * blockDim.x + threadIdx.x;
    if (row >= n) return;
    float sum = 0.f;
    int start = Ap[row];
    int end   = Ap[row + 1];
    for (int k = start; k < end; ++k)
        sum += Ax[k] * x[Aj[k]];
    y[row] = sum;
}

/** y = x（向量拷贝） */
__global__ void vec_copy_kernel(int n, const float* x, float* y)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) y[i] = x[i];
}

/** y = a * x + b * y（就地） */
__global__ void axpby_inplace_kernel(int n, float a, const float* x, float b, float* y)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) y[i] = a * x[i] + b * y[i];
}

/** 计算 V = H * U。U: n x r（列主, ldU=n），V: n x r（列主, ldV=n） */
void compute_HU_on_gpu(const CsrDevice& H, const float* d_U, int ldU,
                       float* d_V, int ldV, int r)
{
    int threads = 256;
    int blocks  = (H.n + threads - 1) / threads;
    for (int j = 0; j < r; ++j) {
        const float* colU = d_U + j * ldU;
        float*       colV = d_V + j * ldV;
        csr_spmv_kernel<<<blocks, threads>>>(H.n, H.Ap, H.Aj, H.Ax, colU, colV);
        CUDA_CHECK(cudaGetLastError());
    }
}

/** A = U^T * V（U,V: n x r，列主；A: r x r，行主/紧凑） */
__global__ void gram_UTV_kernel(int n, const float* U, int ldU,
                                const float* V, int ldV,
                                int r, float* A)
{
    int i = blockIdx.y * blockDim.y + threadIdx.y; // 行
    int j = blockIdx.x * blockDim.x + threadIdx.x; // 列
    if (i >= r || j >= r) return;

    float sum = 0.f;
    for (int k = 0; k < n; ++k)
        sum += U[k + i * ldU] * V[k + j * ldV];
    A[i * r + j] = sum;
}

/** rhs = U^T * vec_r */
__global__ void UT_vec_kernel(int n, const float* U, int ldU, int r,
                              const float* vec_r, float* rhs)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x; // 0..r-1
    if (i >= r) return;
    float sum = 0.f;
    for (int k = 0; k < n; ++k)
        sum += U[k + i * ldU] * vec_r[k];
    rhs[i] = sum;
}

/** 设备端 Cholesky（下三角）分解：A = L L^T，A 原地覆写为 L */
__device__ bool chol_decomp_inplace(float* A, int r)
{
    for (int j = 0; j < r; ++j) {
        float sum = 0.f;
        for (int k = 0; k < j; ++k)
            sum += A[j*r + k] * A[j*r + k];
        float d = A[j*r + j] - sum;
        if (d <= 0.f) return false; // 非正定
        float ljj = sqrtf(d);
        A[j*r + j] = ljj;

        for (int i = j + 1; i < r; ++i) {
            float s = 0.f;
            for (int k = 0; k < j; ++k)
                s += A[i*r + k] * A[j*r + k];
            A[i*r + j] = (A[i*r + j] - s) / ljj;
        }
        // 上三角清零，便于后续只读下三角
        for (int k = j + 1; k < r; ++k) A[j*r + k] = 0.f;
    }
    return true;
}

/** 解 L L^T x = b（单线程执行；共享内存暂存 y） */
__device__ void chol_solve_LLt(const float* L, const float* b, float* x, int r)
{
    extern __shared__ float s[];
    float* y = s;

    // 前代：L y = b
    for (int i = 0; i < r; ++i) {
        float sum = 0.f;
        for (int k = 0; k < i; ++k) sum += L[i*r + k] * y[k];
        y[i] = (b[i] - sum) / L[i*r + i];
    }
    // 回代：L^T x = y
    for (int i = r - 1; i >= 0; --i) {
        float sum = 0.f;
        for (int k = i + 1; k < r; ++k) sum += L[k*r + i] * x[k];
        x[i] = (y[i] - sum) / L[i*r + i];
    }
}

/** 设备端完成：Cholesky + 求解；A 被原地覆写为 L */
__global__ void small_chol_solve_kernel(float* A, float* rhs, float* q, int r)
{
    if (threadIdx.x == 0) {
        bool ok = chol_decomp_inplace(A, r);
        if (!ok) {
            // 轻微对角阻尼以提稳
            for (int i = 0; i < r; ++i) A[i*r + i] += 1e-6f;
            ok = chol_decomp_inplace(A, r);
        }
    }
    __syncthreads();

    extern __shared__ float smem[];
    // q 初始化
    for (int i = threadIdx.x; i < r; i += blockDim.x) q[i] = 0.f;
    __syncthreads();

    // r 通常较小（16~64），单线程前/回代够快且简单
    if (threadIdx.x == 0) chol_solve_LLt(A, rhs, q, r);
}

/** x_out = X + U q */
__global__ void combine_update_kernel(int n, const float* U, int ldU, int r,
                                      const float* q, const float* X, float* x_out)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    float dx = 0.f;
    for (int j = 0; j < r; ++j) dx += U[i + j * ldU] * q[j];
    x_out[i] = X[i] + dx;
}

} // namespace (anonymous)

// =============================== CUDA Entrance ===============================
extern "C" void inexact_solver_cuda(const CsrDevice& H,
                         const float* d_U, int ldU,
                         const float* d_X,
                         const float* d_b,
                         int n, int r,
                         float* d_x_out)
{
    assert(n == H.n);
    assert(r > 0 && r <= R_MAX);

    //device mem
    float *d_HX = nullptr, *d_r = nullptr;
    float *d_V  = nullptr;
    float *d_A  = nullptr;
    float *d_rhs= nullptr;
    float *d_q  = nullptr;

    CUDA_CHECK(cudaMalloc(&d_HX,  n * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_r,   n * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_V,   n * r * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_A,   r * r * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_rhs, r * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_q,   r * sizeof(float)));

    // 1) HX = H * X
    {
        int threads = 256, blocks = (n + threads - 1) / threads;
        csr_spmv_kernel<<<blocks, threads>>>(n, H.Ap, H.Aj, H.Ax, d_X, d_HX);
        CUDA_CHECK(cudaGetLastError());
    }
    // 2) r = b - HX
    {
        int threads = 256, blocks = (n + threads - 1) / threads;
        vec_copy_kernel<<<blocks, threads>>>(n, d_b, d_r);
        CUDA_CHECK(cudaGetLastError());
        axpby_inplace_kernel<<<blocks, threads>>>(n, 1.f, d_r, -1.f, d_HX); // HX = r - HX
        CUDA_CHECK(cudaGetLastError());
        vec_copy_kernel<<<blocks, threads>>>(n, d_HX, d_r);                 // r = b - HX
        CUDA_CHECK(cudaGetLastError());
    }
    // 3) V = H * U
    compute_HU_on_gpu(H, d_U, ldU, d_V, n, r);

    // 4) A = U^T * V
    {
        dim3 threads(16, 16);
        dim3 blocks((r + threads.x - 1) / threads.x,
                    (r + threads.y - 1) / threads.y);
        gram_UTV_kernel<<<blocks, threads>>>(n, d_U, ldU, d_V, n, r, d_A);
        CUDA_CHECK(cudaGetLastError());
    }
    // 5) rhs = U^T * r
    {
        int threads = 256, blocks = (r + threads - 1) / threads;
        UT_vec_kernel<<<blocks, threads>>>(n, d_U, ldU, r, d_r, d_rhs);
        CUDA_CHECK(cudaGetLastError());
    }
    // 6) solve A q = rhs（device Cholesky）
    {
        int threads = 64;
        size_t smem = sizeof(float) * r; // chol_solve_LLt 的中间 y
        small_chol_solve_kernel<<<1, threads, smem>>>(d_A, d_rhs, d_q, r);
        CUDA_CHECK(cudaGetLastError());
    }
    // 7) x = X + U q
    {
        int threads = 256, blocks = (n + threads - 1) / threads;
        combine_update_kernel<<<blocks, threads>>>(n, d_U, ldU, r, d_q, d_X, d_x_out);
        CUDA_CHECK(cudaGetLastError());
    }


    cudaFree(d_HX);
    cudaFree(d_r); 
    cudaFree(d_V);
    cudaFree(d_A);  
    cudaFree(d_rhs);
    cudaFree(d_q);
}
