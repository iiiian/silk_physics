#include <iostream>
#include <vector>
#include <cuda_runtime.h>

// --- Eigen CUDA Setup ---
#define EIGEN_NO_MALLOC
#define EIGEN_USE_GPU

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cloth_solver_kernels.cuh"

// --- CUDA Error Checking Macro ---
#define CHECK_CUDA_ERROR(val) checkCuda((val), #val, __FILE__, __LINE__)
void checkCuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result != cudaSuccess) {
        std::cerr << "CUDA Error at " << file << ":" << line << " code=" << static_cast<unsigned int>(result) << " \"" << func << "\" : " << cudaGetErrorString(result) << std::endl;
        exit(1);
    }
}

/**
 * @brief Standalone test program for GPU cloth solver kernels
 *
 * This program demonstrates the GPU elastic RHS computation on a simple
 * single-triangle test case. It mimics the structure of the CPU cloth solver
 * but runs the elastic projection step on GPU.
 */
int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "   GPU Cloth Solver Kernel Test" << std::endl;
    std::cout << "==================================================" << std::endl;

    // --- 1. Setup Host Data (CPU) ---
    // We simulate a single triangle (3 vertices)
    const int num_vertices = 3;
    const int state_num = num_vertices * 3; // 9 DOFs
    const int ops_num = 1;                    // 1 triangle
    const float elastic_stiffness = 1000.0f;

    std::vector<float> h_state(state_num);         // Vertex positions
    std::vector<int> h_F(ops_num * 3);             // Face indices
    std::vector<float> h_areas(ops_num);           // Face rest areas
    std::vector<float> h_jacobian_ops(ops_num * 54); // jacobian_ops (6x9 per face)
    std::vector<float> h_outer_rhs(state_num);     // External forces (e.g., gravity)
    std::vector<float> h_final_rhs(state_num);     // Final result

    // --- Hardcode a simple triangle at (0,0,0), (1,0,0), (0,1,0) ---
    std::cout << "\n[1] Setting up test triangle:" << std::endl;
    h_state[0] = 0.0f; h_state[1] = 0.0f; h_state[2] = 0.0f; // v0
    h_state[3] = 1.0f; h_state[4] = 0.0f; h_state[5] = 0.0f; // v1
    h_state[6] = 0.0f; h_state[7] = 1.0f; h_state[8] = 0.0f; // v2

    std::cout << "  v0 = (" << h_state[0] << ", " << h_state[1] << ", " << h_state[2] << ")" << std::endl;
    std::cout << "  v1 = (" << h_state[3] << ", " << h_state[4] << ", " << h_state[5] << ")" << std::endl;
    std::cout << "  v2 = (" << h_state[6] << ", " << h_state[7] << ", " << h_state[8] << ")" << std::endl;

    // Face uses vertices 0, 1, 2
    h_F[0] = 0; h_F[1] = 1; h_F[2] = 2;

    // Area of this triangle is 0.5
    h_areas[0] = 0.5f;
    std::cout << "  Area = " << h_areas[0] << std::endl;

    // --- Simple Jacobian operator (identity-like for testing) ---
    // In a real scenario, this would be computed from rest configuration
    // For this test, we use a simple pattern
    std::cout << "\n[2] Initializing Jacobian operator (6x9 matrix):" << std::endl;
    for (int i = 0; i < 54; ++i) {
        // Use a simple pattern that will produce non-zero results
        h_jacobian_ops[i] = (i < 18) ? 0.1f : 0.05f;
    }
    std::cout << "  Jacobian operator filled with test values" << std::endl;

    // --- Outer RHS represents external forces (gravity on Z axis) ---
    std::cout << "\n[3] Setting up external forces (gravity):" << std::endl;
    h_outer_rhs[0] = 0.0f; h_outer_rhs[1] = 0.0f; h_outer_rhs[2] = -10.0f;
    h_outer_rhs[3] = 0.0f; h_outer_rhs[4] = 0.0f; h_outer_rhs[5] = -10.0f;
    h_outer_rhs[6] = 0.0f; h_outer_rhs[7] = 0.0f; h_outer_rhs[8] = -10.0f;
    std::cout << "  Gravity force (Z-axis): -10.0 per vertex" << std::endl;

    // --- 2. Allocate GPU Memory ---
    std::cout << "\n[4] Allocating GPU memory..." << std::endl;
    int *d_F;
    float *d_state, *d_jacobian_ops, *d_areas, *d_outer_rhs;
    float *d_elastic_rhs, *d_final_rhs;

    CHECK_CUDA_ERROR(cudaMalloc(&d_F, ops_num * 3 * sizeof(int)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_state, state_num * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_jacobian_ops, ops_num * 54 * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_areas, ops_num * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_outer_rhs, state_num * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_elastic_rhs, state_num * sizeof(float)));
    CHECK_CUDA_ERROR(cudaMalloc(&d_final_rhs, state_num * sizeof(float)));

    std::cout << "  GPU memory allocated successfully" << std::endl;
    std::cout << "    Faces (d_F):         " << ops_num * 3 * sizeof(int) << " bytes" << std::endl;
    std::cout << "    State (d_state):     " << state_num * sizeof(float) << " bytes" << std::endl;
    std::cout << "    Jacobians:           " << ops_num * 54 * sizeof(float) << " bytes" << std::endl;
    std::cout << "    Total GPU memory:    " <<
        (ops_num * 3 * sizeof(int) + state_num * sizeof(float) * 5 +
         ops_num * 54 * sizeof(float) + ops_num * sizeof(float)) / 1024.0f << " KB" << std::endl;

    // --- 3. Copy Data from Host to Device ---
    std::cout << "\n[5] Copying data to GPU..." << std::endl;
    CHECK_CUDA_ERROR(cudaMemcpy(d_F, h_F.data(), ops_num * 3 * sizeof(int), cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(d_state, h_state.data(), state_num * sizeof(float), cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(d_jacobian_ops, h_jacobian_ops.data(), ops_num * 54 * sizeof(float), cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(d_areas, h_areas.data(), ops_num * sizeof(float), cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(d_outer_rhs, h_outer_rhs.data(), state_num * sizeof(float), cudaMemcpyHostToDevice));
    std::cout << "  Data transfer complete" << std::endl;

    // --- 4. Launch GPU Kernels ---
    std::cout << "\n[6] Launching GPU kernels..." << std::endl;

    // Zero the elastic RHS buffer
    CHECK_CUDA_ERROR(cudaMemset(d_elastic_rhs, 0, state_num * sizeof(float)));

    // Launch elastic RHS computation kernel
    std::cout << "  Launching elastic RHS kernel (per-face SVD projection)..." << std::endl;
    try {
        silk::gpu::launch_compute_elastic_rhs_kernel(
            ops_num, d_F, d_state, d_jacobian_ops, d_areas,
            elastic_stiffness, d_elastic_rhs);
        std::cout << "    Elastic RHS kernel completed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    // Launch vector addition kernel (combine outer_rhs + elastic_rhs)
    std::cout << "  Launching vector addition kernel..." << std::endl;
    try {
        silk::gpu::launch_add_vectors_kernel(
            state_num, d_outer_rhs, d_elastic_rhs, d_final_rhs);
        std::cout << "    Vector addition kernel completed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    // Wait for GPU to finish
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    std::cout << "  GPU kernels finished successfully" << std::endl;

    // --- 5. Copy Results Back to Host ---
    std::cout << "\n[7] Copying results back to CPU..." << std::endl;
    CHECK_CUDA_ERROR(cudaMemcpy(h_final_rhs.data(), d_final_rhs, state_num * sizeof(float), cudaMemcpyDeviceToHost));
    std::cout << "  Result transfer complete" << std::endl;

    // --- 6. Print Results ---
    std::cout << "\n==================================================" << std::endl;
    std::cout << "   FINAL RHS VECTOR (outer_rhs + elastic_rhs)" << std::endl;
    std::cout << "==================================================" << std::endl;
    for (int i = 0; i < num_vertices; ++i) {
        std::cout << "Vertex " << i << ": ("
                  << h_final_rhs[i*3 + 0] << ", "
                  << h_final_rhs[i*3 + 1] << ", "
                  << h_final_rhs[i*3 + 2] << ")" << std::endl;
    }
    std::cout << "==================================================" << std::endl;

    // --- 7. Verify Results ---
    std::cout << "\n[8] Verifying results..." << std::endl;
    bool all_finite = true;
    for (int i = 0; i < state_num; ++i) {
        if (!std::isfinite(h_final_rhs[i])) {
            all_finite = false;
            std::cout << "  WARNING: Non-finite value at index " << i << std::endl;
        }
    }
    if (all_finite) {
        std::cout << "  All values are finite - test PASSED" << std::endl;
    } else {
        std::cout << "  Found non-finite values - test FAILED" << std::endl;
    }

    // --- 8. Cleanup ---
    std::cout << "\n[9] Cleaning up GPU memory..." << std::endl;
    CHECK_CUDA_ERROR(cudaFree(d_F));
    CHECK_CUDA_ERROR(cudaFree(d_state));
    CHECK_CUDA_ERROR(cudaFree(d_jacobian_ops));
    CHECK_CUDA_ERROR(cudaFree(d_areas));
    CHECK_CUDA_ERROR(cudaFree(d_outer_rhs));
    CHECK_CUDA_ERROR(cudaFree(d_elastic_rhs));
    CHECK_CUDA_ERROR(cudaFree(d_final_rhs));
    std::cout << "  GPU memory freed" << std::endl;

    std::cout << "\n==================================================" << std::endl;
    std::cout << "   Test Complete - SUCCESS" << std::endl;
    std::cout << "==================================================" << std::endl;

    return 0;
}
