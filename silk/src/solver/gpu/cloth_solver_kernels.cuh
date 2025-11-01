#pragma once

/**
 * @file cloth_solver_kernels.cuh
 * @brief CUDA kernels for GPU-accelerated cloth simulation
 *
 * This header declares the CUDA kernels and host wrapper functions for the
 * elastic RHS computation step in the projective dynamics cloth solver.
 */

namespace silk {
namespace gpu {

/**
 * @brief Launch the elastic RHS computation kernel
 *
 * Computes per-face elastic force contributions using projective dynamics:
 * - Per-thread SVD projection of deformation gradient
 * - Singular value clamping for limited stretch
 * - Atomic accumulation to global RHS vector
 *
 * @param ops_num Number of triangle faces
 * @param d_F Device pointer to face indices [ops_num * 3]
 * @param d_state Device pointer to vertex positions [state_num]
 * @param d_jacobian_ops Device pointer to 6x9 Jacobian matrices [ops_num * 54]
 * @param d_areas Device pointer to per-face rest areas [ops_num]
 * @param elastic_stiffness Material stiffness parameter
 * @param d_elastic_rhs Device pointer to output RHS [state_num] (must be
 * zero-initialized)
 * @param block_size CUDA block size (default: 256)
 *
 * @throws std::runtime_error if kernel launch fails
 */
void launch_compute_elastic_rhs_kernel(int ops_num, const int* d_F,
                                       const float* d_state,
                                       const float* d_jacobian_ops,
                                       const float* d_areas,
                                       float elastic_stiffness,
                                       float* d_elastic_rhs,
                                       int block_size = 256);

/**
 * @brief Launch the vector addition kernel
 *
 * Performs element-wise addition: result = a + b
 * Used to combine outer_rhs with elastic_rhs
 *
 * @param n Vector length
 * @param a Device pointer to first input vector
 * @param b Device pointer to second input vector
 * @param result Device pointer to output vector
 * @param block_size CUDA block size (default: 256)
 *
 * @throws std::runtime_error if kernel launch fails
 */
void launch_add_vectors_kernel(int n, const float* a, const float* b,
                                float* result, int block_size = 256);

}  // namespace gpu
}  // namespace silk
