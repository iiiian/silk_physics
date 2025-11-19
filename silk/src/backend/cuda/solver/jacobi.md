# Block-Jacobi CUDA Kernel in Silk

This note documents the CUDA Jacobi solver used in `a_jacobi_solver.cu`, and how it leverages the connectivity-based vertex ordering.

## Background

We solve linear systems of the form

> \[(D + R) x = b\]

where:

- `D` is the diagonal part of the system matrix.
- `R` is the strictly off-diagonal part (stored as `CSRMatrix` in row-major CSR).
- `x` is the unknown stacked DOF vector (size `state_num`).
- `b` is the right-hand side (outer + elastic + barrier terms).

The matrix and vectors live in a **permuted DOF order**:

- Vertices are reordered using a KDTree on rest positions.
- DOFs for vertex `i` are always `[3*i, 3*i+1, 3*i+2]` in this permuted space.
- All solver-side data (`d_R`, `d_D`, `d_DB`, etc.) are assembled in this order.

The Jacobi update for row `i` is

> \[x_i^{(k+1)} = (b_i - \sum_{j} R_{ij} x_j^{(k)}) / D_{ii}\]

## Block-Jacobi Kernel Design

The main kernel is:

```cpp
__global__ void compute_x_block_kernel(
    int n,
    CSRMatrixView d_R,
    const float* d_D,
    const float* d_rhs,
    const float* d_x_in,
    float* d_x_out,
    int inner_iters);
```

Key properties:

- **Row partitioning:**  
  - 1D grid; each block owns a contiguous range of rows.  
  - Global row index: `row = blockIdx.x * blockDim.x + threadIdx.x`.  
  - Block range: `[block_row_start, block_row_start + blockDim.x)`.

- **Shared memory:**  
  - Each block allocates `2 * blockDim.x` floats of dynamic shared memory:  
    - `s_curr[blockDim.x]` – current local values for the block.  
    - `s_next[blockDim.x]` – next iteration's local values.
  - Initialised from `d_x_in` for owned rows.

- **Inner iterations (`inner_iters`):**  
  Within each block we run several Jacobi-like sweeps:

  ```cpp
  for (int it = 0; it < inner_iters; ++it) {
    // For each owned row (global_row < n):
    //   accu = sum_j R[row,j] * x_j
    //   x_j comes from:
    //     - s_curr[local_col] if neighbor j is inside this block's range
    //     - d_x_in[col]       if neighbor j belongs to another block
    //   s_next[local_id] = (rhs[row] - accu) / D[row];

    __syncthreads();
    swap(s_curr, s_next);
    __syncthreads();
  }
  ```

  - **Local neighbors** see updated values across the inner iterations.
  - **External neighbors** always see the frozen snapshot in `d_x_in`.
  - This is a **block-Jacobi / block Gauss–Seidel hybrid**:
    - Pure Jacobi across blocks.
    - Gauss–Seidel-like within a block.

- **Write-back:**  
  After `inner_iters` sweeps, each thread writes `s_curr[local_id]` back to `d_x_out[row]` (if `row < n`).

## Host-Side Solver (`a_jacobi`)

`a_jacobi` manages two device buffers and the residual checks:

```cpp
DVector<float> d_x_buffer(n);
float* d_x_prev = d_x;         // previous iterate
float* d_x_next = d_x_buffer;  // next iterate
```

- **Per-iteration launch:**

  ```cpp
  constexpr int ACCUM_RUN = 4; // inner sweeps per kernel launch

  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     compute_x_block_kernel, 0, 0);
  int grid_size = (n + block_size - 1) / block_size;
  size_t shmem_bytes = 2 * block_size * sizeof(float);

  for (int iter = 0; iter < max_iter; iter += ACCUM_RUN) {
    compute_x_block_kernel<<<grid_size, block_size, shmem_bytes>>>(
        n, d_R.get_view(), d_D, d_rhs, d_x_prev, d_x_next, ACCUM_RUN);
    cudaDeviceSynchronize();

    // d_x_next now holds the newest iterate; swap so d_x_prev is always "current".
    std::swap(d_x_prev, d_x_next);

    // Compute Linf distance between latest and previous iterate.
    Linf_dist = compute_Linf_dist(n, d_x_prev, d_x_next, d_x_next);
    ...
  }
  ```

  - After each launch:
    - `d_x_prev` contains the **newest** iterate.
    - `d_x_next` contains the previous iterate and is used as scratch.

- **Residual check and termination:**

  - `compute_Linf_dist` computes `max_i |x_prev[i] - x_next[i]|`, writing per-entry diffs into `d_x_next`.
  - On convergence:

    ```cpp
    if (d_x_prev != d_x) {
      cudaMemcpy(d_x, d_x_prev, n * sizeof(float),
                 cudaMemcpyDeviceToDevice);
    }
    ```

  - On failure exit, the last iterate in `d_x_prev` is also copied back to `d_x`.

## Rationale

- **Why block-Jacobi?**
  - The original Jacobi implementation launched one kernel per sweep.
  - For the typical `state_num` in cloth, kernel launch overhead dominated.
  - Fusing multiple sweeps per launch improves arithmetic intensity and reduces launch overhead.

- **Why it works well with the vertex permutation:**
  - The KDTree-based vertex ordering tends to group topologically nearby vertices into contiguous index ranges.
  - Mapping these ranges to blocks means many neighbors of a row reside in the same block, so inner iterations can reuse updated values via shared memory.
  - Neighbors in other blocks still use the frozen snapshot, preserving Jacobi-like behavior across blocks and keeping the update inexpensive.

This design keeps the global solver interface unchanged while significantly reducing launch overhead and improving convergence per outer loop step, especially on meshes where the permutation clusters connected vertices. 

