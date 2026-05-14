#pragma once

#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"

namespace silk::cuda::solver {

class ClothADMMHelper {
 private:
  // y := in-plane elastic aux state.
  Buf<float> y_;
  Buf<float> uy_;
  Buf<float> jacobians_;
  // z := bending aux state.
  Buf<float> z_;
  Buf<float> uz_;
  Buf<float> laplacians_;
  // cusparse related.
  CuSparseHandle cusparse_handle_;
  Buf<char> cusparse_workspace_;
  // Other tmp.
  Buf<float> float_tmp;

 public:
  ClothADMMHelper() = default;
  ClothADMMHelper(int vert_num, int face_num, CudaRuntime rt);

  void reset_aux_lagrange_mul(CudaRuntime rt);

  void update_aux_var_and_lagrange_mul(const ClothAssemblyL1Cache& l1_cache,
                                       ctd::span<const float> state,
                                       CudaRuntime rt);

  void solve_main_var(const ClothAssemblyL1Cache& l1_cache,
                      ctd::span<const float> extern_lhs,
                      ctd::span<const float> extern_rhs,
                      ctd::span<const float> inertia_mod,
                      ctd::span<float> state, CudaRuntime rt);
};

}  // namespace silk::cuda::solver
