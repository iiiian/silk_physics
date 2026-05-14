#include "backend/cuda/solver/cloth_admm_helper.cuh"

#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/svd.cuh"

namespace silk::cuda::solver {

namespace {

// clang-format off
__global__ void solve_and_update_elastic_aux(
    int face_num,
    float elastic_stiffness,
    float penalty,
    ctd::span<const int> faces,
    ctd::span<const float> position,
    ctd::span<const float> weighted_jacobian_ops,
    ctd::span<float> jacobians,
    ctd::span<float> lagrange_mul,
    ctd::span<float> aux_var)
// clang-format on
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= face_num) {
    return;
  }

  // Gather position x.
  Mat<float, 9, 1> x;
  for (int i = 0; i < 3; ++i) {
    int x_offset = 3 * faces[tid * 3 + i];
#pragma unroll
    for (int j = 0; j < 3; ++j) {
      x(3 * i + j) = position[x_offset + j];
    }
  }
  // Gather lagrange multipler u.
  Mat<float, 6, 1> u;
  int u_offset = 6 * tid;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    u(i) = lagrange_mul[u_offset + i];
  }
  // Gather jacobian op.
  Mat<float, 6, 9> jop;
  int jop_offset = 54 * tid;
  for (int i = 0; i < 54; ++i) {
    jop.data[i] = weighted_jacobian_ops[jop_offset + i];
  }

  // Compute Sx.
  auto Sx = mat_mul(jop, x);
  int jacobian_offset = tid * 6;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    jacobians[jacobian_offset + i] = Sx(i);
  }

  // Compute aux variable.
  auto y = axpby(1.0, Sx, 1.0 / penalty, u);
  // SVD decomposition D = U S V^T via svd32 (backed by the 3x3 ref SVD).
  Mat32f U;
  Mat22f S = Mat22f::zeros();
  Mat22f V;
  svd32(
      // input D encoded as [d11,d21,d31,d12,d22,d32]
      y(0), y(1), y(2), y(3), y(4), y(5),
      // output U (3x2, column-major)
      U(0, 0), U(1, 0), U(2, 0), U(0, 1), U(1, 1), U(2, 1),
      // output singular values
      S(0, 0), S(1, 1),
      // output V (2x2, column-major)
      V(0, 0), V(1, 0), V(0, 1), V(1, 1));
  S(0, 0) = ctd::clamp(S(0, 0), 0.9f, 1.1f);
  S(1, 1) = ctd::clamp(S(1, 1), 0.9f, 1.1f);
  // proj = U S' V^T where S' is clamped.
  Mat32f proj = mat_mul(U, mat_mul(S, V.view().transpose()));
  float r = elastic_stiffness / (elastic_stiffness + penalty);
  Mat<float, 6, 1> aux = axpby(1 - r, y, r, proj.view().vectorize());
  // Write to global mem.
  int aux_var_offset = tid * 6;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    aux_var[aux_var_offset + i] = aux(i);
  }

  // Update lagrange multipliers.
  auto delta = axpby(penalty, Sx, -penalty, aux);
  int mul_offset = tid * 6;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    lagrange_mul[mul_offset + i] = u(i) + delta(i);
  }
}

// clang-format off
void solve_and_update_bending_aux(
    int vert_num,
    float bending_stiffness,
    float penalty,
    ctd::span<const float> position,
    BSRView weighted_laplacian_ops,
    ctd::span<const float> rest_curvature,
    const CuSparseHandle& cusparse_handle,
    cu::device_buffer<char>& cusparse_workspace,
    ctd::span<float> laplacians,
    ctd::span<float> lagrange_mul,
    ctd::span<float> aux_var,
    CudaRuntime rt)
// clang-format on
{
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_lap{weighted_laplacian_ops};
  CuSparseConstVec cusparse_x{position};
  CuSparseVec cusparse_Sx{laplacians};

  // Compute Sx.
  float alpha = 1.0;
  float beta = 0.0;
  size_t workspace_size;
  cusparseSpMV_bufferSize(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE,
                          &alpha, cusparse_lap.raw, cusparse_x.raw, &beta,
                          cusparse_Sx.raw, CUDA_R_32F,
                          CUSPARSE_SPMV_ALG_DEFAULT, &workspace_size);
  if (cusparse_workspace.size() < workspace_size) {
    cusparse_workspace = alloc<char>(rt, workspace_size);
  }
  cusparseSpMV(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
               cusparse_lap.raw, cusparse_x.raw, &beta, cusparse_Sx.raw,
               CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT,
               cusparse_workspace.data());

  // Compute aux variables.
  float r = bending_stiffness / (bending_stiffness + penalty);
  auto compute_aux_var = [proj = rest_curvature, lagrange_mul, penalty, r,
                          laplacians, aux_var] __device__(int i) {
    float Sx = laplacians[i];
    float y = Sx + lagrange_mul[i] / penalty;
    aux_var[i] = (1.0 - r) * y + r * proj[i];
  };
  cub::DeviceFor::Bulk(aux_var.size(), compute_aux_var, rt.stream.get());

  // Update lagrange multiplers.
  auto update_mul = [lagrange_mul, laplacians, aux_var,
                     penalty] __device__(int i) {
    float delta = penalty * (laplacians[i] - aux_var[i]);
    lagrange_mul[i] += delta;
  };
  cub::DeviceFor::Bulk(lagrange_mul.size(), update_mul, rt.stream.get());
}

__global__ void assemble_inertia(int vert_num, float dt,
                                 ctd::span<const float> mass,
                                 ctd::span<const float> x,
                                 ctd::span<const float> inertia_mod,
                                 ctd::span<float> lhs_diag) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= vert_num) {
    return;
  }

  lhs_diag[tid] += mass[tid] / (dt * dt) * x[tid] + inertia_mod[tid];
}

// clang-format off
__global__ void assemble_elastic_rhs(
    int face_num,
    float penalty,
    ctd::span<const int> faces,
    ctd::span<const float> weighted_jacobian_ops,
    ctd::span<const float> lagrange_mul,
    ctd::span<const float> aux_var,
    ctd::span<float> rhs)
// clang-format on
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= face_num) {
    return;
  }

  // Gather jacobian op.
  Mat<float, 6, 9> jop;
  int jop_offset = 54 * tid;
  for (int i = 0; i < 54; ++i) {
    jop.data[i] = weighted_jacobian_ops[jop_offset + i];
  }
  // Gather lagrange multipler u.
  Mat<float, 6, 1> u;
  int u_offset = 6 * tid;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    u(i) = lagrange_mul[u_offset + i];
  }
  // Gather aux variable x.
  Mat<float, 6, 1> x;
  int x_offset = 6 * tid;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    x(i) = aux_var[u_offset + i];
  }

  auto S_tr = jop.view().transpose();
  auto tmp = Mat<float, 9, 1>::zeros();
  vadd(tmp, mat_mul(S_tr, u));
  vadd(tmp, ax(penalty, mat_mul(S_tr, x)));

  // Write to global mem.
  for (int i = 0; i < 3; ++i) {
    int rhs_offset = 3 * faces[tid * 3 + i];
#pragma unroll
    for (int j = 0; j < 3; ++j) {
      rhs[rhs_offset + j] += tmp(3 * i + j);
    }
  }
}

void assemble_bending_rhs(float penalty, BSRView weighted_laplacian_ops,
                          ctd::span<const float> lagrange_mul,
                          ctd::span<const float> aux_var,
                          const CuSparseHandle& cusparse_handle,
                          cu::device_buffer<char>& cusparse_workspace,
                          cu::device_buffer<float>& tmp, ctd::span<float> rhs,
                          CudaRuntime rt) {
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_lap{weighted_laplacian_ops};
  CuSparseConstVec cusparse_x{aux_var};
  CuSparseConstVec cusparse_u{lagrange_mul};

  if (tmp.size() != rhs.size()) {
    tmp = alloc<float>(rt, rhs.size());
  }
  CuSparseVec cusparse_tmp{tmp};

  // Compute Su.
  float alpha = 1.0;
  float beta = 0.0;
  size_t workspace_size;
  cusparseSpMV_bufferSize(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE,
                          &alpha, cusparse_lap.raw, cusparse_u.raw, &beta,
                          cusparse_tmp.raw, CUDA_R_32F,
                          CUSPARSE_SPMV_ALG_DEFAULT, &workspace_size);
  if (cusparse_workspace.size() < workspace_size) {
    cusparse_workspace = alloc<char>(rt, workspace_size);
  }
  cusparseSpMV(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
               cusparse_lap.raw, cusparse_u.raw, &beta, cusparse_tmp.raw,
               CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT,
               cusparse_workspace.data());

  // Compute Sx.
  alpha = penalty;
  beta = 1.0;
  cusparseSpMV_bufferSize(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE,
                          &alpha, cusparse_lap.raw, cusparse_x.raw, &beta,
                          cusparse_tmp.raw, CUDA_R_32F,
                          CUSPARSE_SPMV_ALG_DEFAULT, &workspace_size);
  if (cusparse_workspace.size() < workspace_size) {
    cusparse_workspace = alloc<char>(rt, workspace_size);
  }
  cusparseSpMV(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
               cusparse_lap.raw, cusparse_u.raw, &beta, cusparse_tmp.raw,
               CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT,
               cusparse_workspace.data());

  auto accum = [rhs, tmp_span = ctd::span<const float>{tmp}] __device__(int i) {
    rhs[i] += tmp_span[i];
  };
  cub::DeviceFor::Bulk(rhs.size(), accum, rt.stream.get());
}

}  // namespace

ClothADMMHelper::ClothADMMHelper(int vert_num, int face_num, CudaRuntime rt) {
  int jacobian_dof = 6 * face_num;
  y_ = alloc<float>(rt, jacobian_dof);
  uy_ = alloc<float>(rt, jacobian_dof, 0);
  jacobians_ = alloc<float>(rt, jacobian_dof);
  // Curvature is per vertex.
  int curvature_dof = vert_num;
  z_ = alloc<float>(rt, curvature_dof);
  uz_ = alloc<float>(rt, curvature_dof, 0);
  laplacians_ = alloc<float>(rt, curvature_dof);
}

void ClothADMMHelper::reset_aux_lagrange_mul(CudaRuntime rt) {
  cu::fill_bytes(rt.stream, *uy_, 0);
  cu::fill_bytes(rt.stream, *uz_, 0);
}

void ClothADMMHelper::update_aux_var_and_lagrange_mul(
    const ClothAssemblyL1Cache& l1_cache, ctd::span<const float> state,
    CudaRuntime rt) {
  int grid_num = div_round_up(l1_cache.face_num, 128);
  solve_and_update_elastic_aux<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.face_num, l1_cache.elastic_stiffness, l1_cache.penalty,
      *l1_cache.faces, state, *l1_cache.weighted_jacobian_ops, *jacobians_,
      *uy_, *y_);

  solve_and_update_bending_aux(
      l1_cache.vert_num, l1_cache.bending_stiffness, l1_cache.penalty, state,
      l1_cache.weighted_laplacian_ops.view(), *l1_cache.C0, cusparse_handle_,
      *cusparse_workspace_, *laplacians_, *uz_, *z_, rt);
}

void ClothADMMHelper::solve_main_var(const ClothAssemblyL1Cache& l1_cache,
                                     ctd::span<const float> extern_lhs,
                                     ctd::span<const float> extern_rhs,
                                     ctd::span<const float> inertia_mod,
                                     ctd::span<float> state, CudaRuntime rt) {
  auto lhs_diag = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_lhs, lhs_diag);
  auto rhs = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_rhs, rhs);

  int grid_num = div_round_up(l1_cache.state_num, 128);
  assemble_inertia<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.vert_num, l1_cache.dt, *l1_cache.mass, state, inertia_mod,
      lhs_diag);

  grid_num = div_round_up(l1_cache.face_num, 128);
  assemble_elastic_rhs<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.face_num, l1_cache.penalty, *l1_cache.faces,
      *l1_cache.weighted_jacobian_ops, *uy_, *y_, rhs);

  if (!float_tmp) {
    float_tmp = alloc<float>(rt, rhs.size());
  }
  assemble_bending_rhs(l1_cache.penalty, l1_cache.weighted_laplacian_ops.view(),
                       *uz_, *z_, cusparse_handle_, *cusparse_workspace_,
                       *float_tmp, rhs, rt);
}

}  // namespace silk::cuda::solver
