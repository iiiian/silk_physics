#pragma once

#include <cstdint>
#include <vector>
#include <Eigen/SparseCore>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {
    /// Host CSR adjacency for graph partitioning, built using CUDA.
    /// Self-excluding, with symmetrized + quantized positive integer weights.
    class BSRAdjacency {
    public:
        BSRAdjacency() = default;
        BSRAdjacency(const Eigen::SparseMatrix<float> &a, int block_dim, CudaRuntime rt);

        std::vector<int> row_ptr;
        std::vector<int> cols;
        std::vector<int64_t> weights;
    };

} // namespace silk::cuda
