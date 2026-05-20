#pragma once

#include <cuda/buffer>
#include <cuda/std/span>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

class EqualityConstraints {
 private:
  int state_num;
  int active_dof;
  cu::device_buffer<bool> indicator;
  cu::device_buffer<float> target;
  cu::device_buffer<float> lagrange_mul;

 public:
  float init_lagrange_mul;
  float max_lagrange_mul;
  float init_penalty;
  float penalty;
  // TODO: maybe update penalty?

  EqualityConstraints(int state_num, float init_penalty,
                      float init_lagrange_mul, float max_lagrange_mul,
                      cu::device_buffer<bool> indicator,
                      cu::device_buffer<float> target, CudaRuntime rt);

  int get_active_dof() const { return active_dof; }
  void merge(const EqualityConstraints& other, CudaRuntime rt);
  void reset_lagrange_mul(CudaRuntime rt);
  void update_lagrange_mul(ctd::span<const float> state, CudaRuntime rt);
  void eval(ctd::span<float> lhs_diag, ctd::span<float> rhs,
            CudaRuntime rt) const;
  void accum_primal_residual(ctd::span<const float> state,
                             ctd::span<float> primal_norm2,
                             ctd::span<float> primal_scale_x2,
                             ctd::span<float> primal_scale_target2,
                             CudaRuntime rt) const;
  void enforce(ctd::span<float> state, CudaRuntime rt);
};

}  // namespace silk::cuda
