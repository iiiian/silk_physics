#include "backend/cuda/solver/cloth_admm_helper.cuh"

#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/inner_product.cuh"
#include "backend/cuda/solver/svd.cuh"

namespace silk::cuda {

namespace {

// clang-format off
__global__ void solve_and_update_elastic_aux(
    int face_num,
    float max_lagrange_mul,
    float elastic_stiffness,
    float penalty,
    ctd::span<const int> faces,
    ctd::span<const float> position,
    ctd::span<const float> jacobian_ops,
    ctd::span<const float> area_sqrt,
    ctd::span<float> lagrange_mul,
    ctd::span<float> aux_var,
    ctd::span<float> primal_norm2,
    ctd::span<float> primal_scale_x2,
    ctd::span<float> primal_scale_aux2,
    ctd::span<float> dual_residual)
// clang-format on
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float local_primal_norm2 = 0.0f;
  float local_primal_scale_x2 = 0.0f;
  float local_primal_scale_aux2 = 0.0f;

  if (tid < face_num) {
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
    // Gather old aux variable.
    Mat<float, 6, 1> old_aux;
    int aux_var_offset = tid * 6;
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      old_aux(i) = aux_var[aux_var_offset + i];
    }
    // Gather jacobian op.
    Mat<float, 6, 9> jop;
    int jop_offset = 54 * tid;
    for (int i = 0; i < 54; ++i) {
      jop.data[i] = jacobian_ops[jop_offset + i];
    }
    // Gather weight.
    float w = area_sqrt[tid];

    // Compute Sx.
    auto Sx = mat_mul(jop, x);

    // Compute aux variable.
    auto y = axpby(1.0, Sx, 1.0f / (penalty * w), u);
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
    Mat<float, 6, 1> proj_vec;
    proj_vec(0) = proj(0, 0);
    proj_vec(1) = proj(1, 0);
    proj_vec(2) = proj(2, 0);
    proj_vec(3) = proj(0, 1);
    proj_vec(4) = proj(1, 1);
    proj_vec(5) = proj(2, 1);
    float r = elastic_stiffness / (elastic_stiffness + penalty);
    Mat<float, 6, 1> aux = axpby(1 - r, y, r, proj_vec);

    // Per face primal residual.
    // r = W(Sx - z)
    auto primal = ax(w, vsub(Sx, aux));
    local_primal_norm2 = squared_norm(primal);
    local_primal_scale_x2 = squared_norm(ax(w, Sx));
    local_primal_scale_aux2 = squared_norm(ax(w, aux));

    // Per state dof dual residual.
    // r = rho * S^T W^T W (z - z_old)
    auto S_tr = jop.view().transpose();
    auto dual = ax(penalty * w * w, mat_mul(S_tr, vsub(aux, old_aux)));
    for (int i = 0; i < 3; ++i) {
      int out_offset = 3 * faces[tid * 3 + i];
#pragma unroll
      for (int j = 0; j < 3; ++j) {
        atomicAdd(dual_residual.data() + out_offset + j, dual(3 * i + j));
      }
    }

    // Write aux var.
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      aux_var[aux_var_offset + i] = aux(i);
    }

    // Update lagrange multipliers.
    auto delta = axpby(w * penalty, Sx, -w * penalty, aux);
    int mul_offset = tid * 6;
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      lagrange_mul[mul_offset + i] =
          ctd::clamp(u(i) + delta(i), -max_lagrange_mul, max_lagrange_mul);
    }
  }

  // Accumulate primal residual norm2.
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage reduce_tmp;
  float reduced = BlockReduce(reduce_tmp).Sum(local_primal_norm2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_norm2.data(), reduced);
  }
  __syncthreads();
  reduced = BlockReduce(reduce_tmp).Sum(local_primal_scale_x2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_scale_x2.data(), reduced);
  }
  __syncthreads();
  reduced = BlockReduce(reduce_tmp).Sum(local_primal_scale_aux2);
  if (threadIdx.x == 0) {
    atomicAdd(primal_scale_aux2.data(), reduced);
  }
}

// clang-format off
[[maybe_unused]] void solve_and_update_bending_aux(
    int vert_num,
    float max_lagrange_mul,
    float bending_stiffness,
    float penalty,
    ctd::span<const float> position,
    BSRView weighted_laplacian_ops,
    ctd::span<const float> rest_curvature,
    const CuSparseHandle& cusparse_handle,
    cu::device_buffer<char>& cusparse_workspace,
    cu::device_buffer<float>& tmp,
    ctd::span<float> lagrange_mul,
    ctd::span<float> aux_var,
    CudaRuntime rt)
// clang-format on
{
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_lap{weighted_laplacian_ops};
  CuSparseConstVec cusparse_x{position};

  if (tmp.size() < position.size()) {
    tmp = alloc<float>(rt, position.size());
  }
  CuSparseVec cusparse_Sx{tmp};

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
  ctd::span<const float> Sx{tmp};
  auto compute_aux_var = [proj = rest_curvature, Sx, lagrange_mul, penalty, r,
                          aux_var] __device__(int i) {
    float y = Sx[i] + lagrange_mul[i] / penalty;
    aux_var[i] = (1.0 - r) * y + r * proj[i];
  };
  cub::DeviceFor::Bulk(aux_var.size(), compute_aux_var, rt.stream.get());

  // Update lagrange multiplers.
  auto update_mul = [lagrange_mul, Sx, aux_var, penalty,
                     max_lagrange_mul] __device__(int i) {
    float delta = penalty * (Sx[i] - aux_var[i]);
    lagrange_mul[i] = min(lagrange_mul[i] + delta, max_lagrange_mul);
  };
  cub::DeviceFor::Bulk(lagrange_mul.size(), update_mul, rt.stream.get());
}

__global__ void assemble_inertia(float dt, ctd::span<const float> mass,
                                 ctd::span<const float> x,
                                 ctd::span<const float> inertia_mod,
                                 ctd::span<float> lhs_diag,
                                 ctd::span<float> rhs) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= lhs_diag.size()) {
    return;
  }

  lhs_diag[tid] += mass[tid] / (dt * dt);
  rhs[tid] -= mass[tid] * inertia_mod[tid];
}

__global__ void add_scalar_kernel(ctd::span<const float> value,
                                  ctd::span<float> out) {
  if (threadIdx.x == 0 && blockIdx.x == 0) {
    atomicAdd(out.data(), value[0]);
  }
}

// clang-format off
__global__ void assemble_elastic_rhs(
    int face_num,
    float penalty,
    ctd::span<const int> faces,
    ctd::span<const float> jacobian_ops,
    ctd::span<const float> area_sqrt,
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
    jop.data[i] = jacobian_ops[jop_offset + i];
  }
  // Gather lagrange multipler u.
  Mat<float, 6, 1> u;
  int u_offset = 6 * tid;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    u(i) = lagrange_mul[u_offset + i];
  }
  // Gather aux variable z.
  Mat<float, 6, 1> z;
  int x_offset = 6 * tid;
#pragma unroll
  for (int i = 0; i < 6; ++i) {
    z(i) = aux_var[x_offset + i];
  }
  // Gather weight.
  float w = area_sqrt[tid];

  auto S_tr = jop.view().transpose();
  auto tmp = Mat<float, 9, 1>::zeros();
  tmp = vadd(tmp, ax(-w, mat_mul(S_tr, u)));
  tmp = vadd(tmp, ax(w * w * penalty, mat_mul(S_tr, z)));

  // Write to global mem.
  for (int i = 0; i < 3; ++i) {
    int rhs_offset = 3 * faces[tid * 3 + i];
#pragma unroll
    for (int j = 0; j < 3; ++j) {
      atomicAdd(rhs.data() + rhs_offset + j, tmp(3 * i + j));
    }
  }
}

[[maybe_unused]] void assemble_bending_rhs(
    float penalty, BSRView weighted_laplacian_ops,
    ctd::span<const float> lagrange_mul, ctd::span<const float> aux_var,
    const CuSparseHandle& cusparse_handle,
    cu::device_buffer<char>& cusparse_workspace, cu::device_buffer<float>& tmp,
    ctd::span<float> rhs, CudaRuntime rt) {
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_lap{weighted_laplacian_ops};
  CuSparseConstVec cusparse_x{aux_var};
  CuSparseConstVec cusparse_u{lagrange_mul};

  if (tmp.size() < rhs.size()) {
    tmp = alloc<float>(rt, rhs.size());
  }
  CuSparseVec cusparse_tmp{tmp};

  // Compute -Su.
  float alpha = -1.0;
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

  // Compute penalty * Sx.
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
               cusparse_lap.raw, cusparse_x.raw, &beta, cusparse_tmp.raw,
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
  // Curvature is per vertex coordinate.
  int curvature_dof = 3 * vert_num;
  z_ = alloc<float>(rt, curvature_dof);
  uz_ = alloc<float>(rt, curvature_dof, 0);

  cusparse_workspace_ = alloc<char>(rt, 0);
  float_tmp_ = alloc<float>(rt, 0);
}

void ClothADMMHelper::reset_aux_lagrange_mul(CudaRuntime rt) {
  cu::fill_bytes(rt.stream, *y_, 0);
  cu::fill_bytes(rt.stream, *uy_, 0);
  cu::fill_bytes(rt.stream, *z_, 0);
  cu::fill_bytes(rt.stream, *uz_, 0);
}

void ClothADMMHelper::update_aux_var_and_lagrange_mul(
    float max_lagrange_mul, const ClothAssemblyL1Cache& l1_cache,
    ctd::span<const float> state, ctd::span<float> primal_residual_norm2,
    ctd::span<float> primal_scale_x2, ctd::span<float> primal_scale_aux2,
    ctd::span<float> dual_residual, CudaRuntime rt) {
  int grid_num = div_round_up(l1_cache.face_num, 128);
  // clang-format off
  solve_and_update_elastic_aux<<<grid_num, 128, 0, rt.stream.get()>>>(
        l1_cache.face_num,
        max_lagrange_mul,
        l1_cache.elastic_stiffness,
        l1_cache.penalty,
        *l1_cache.faces,
        state,
        *l1_cache.jacobian_ops,
        *l1_cache.area_sqrt,
        *uy_,
        *y_,
        primal_residual_norm2,
        primal_scale_x2,
        primal_scale_aux2,
        dual_residual);
  // clang-format on

  // solve_and_update_bending_aux(
  //     l1_cache.vert_num, max_lagrange_mul, l1_cache.bending_stiffness,
  //     l1_cache.penalty, state, l1_cache.weighted_laplacian_ops.view(),
  //     *l1_cache.C0, cusparse_handle_, *cusparse_workspace_, *float_tmp_,
  //     *uz_, *z_, rt);
}

void ClothADMMHelper::solve_main_var(
    float rel_tol, float abs_tol, const ClothAssemblyL1Cache& l1_cache,
    bool is_lhs_changed, ctd::span<const float> extern_lhs,
    ctd::span<const float> extern_rhs, ctd::span<const float> inertia_mod,
    ctd::span<float> state, ctd::span<float> rhs_norm2, CudaRuntime rt) {
  auto lhs_diag = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_lhs, lhs_diag);
  auto rhs = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_rhs, rhs);

  int grid_num = div_round_up(l1_cache.state_num, 128);
  assemble_inertia<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.dt, *l1_cache.mass, state, inertia_mod, lhs_diag, rhs);

  grid_num = div_round_up(l1_cache.face_num, 128);
  assemble_elastic_rhs<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.face_num, l1_cache.penalty, *l1_cache.faces,
      *l1_cache.jacobian_ops, *l1_cache.area_sqrt, *uy_, *y_, rhs);

  if (!float_tmp_) {
    float_tmp_ = alloc<float>(rt, rhs.size());
  }
  // assemble_bending_rhs(l1_cache.penalty,
  //                      l1_cache.weighted_laplacian_ops.view(), *uz_, *z_,
  //                      cusparse_handle_, *cusparse_workspace_, *float_tmp_,
  //                      rhs, rt);

  auto scalar_local_rhs_norm2 = alloc<float>(rt, 1);
  inner_product(rhs, rhs, scalar_local_rhs_norm2, rt);
  add_scalar_kernel<<<1, 1, 0, rt.stream.get()>>>(scalar_local_rhs_norm2,
                                                  rhs_norm2);

  DynamicBSRView dyn_A{lhs_diag, l1_cache.weighted_AA.view()};
  if (is_lhs_changed) {
    linear_solver_.factorize(dyn_A, *l1_cache.part_offsets, rt);
  }
  linear_solver_.abs_tol = abs_tol;
  linear_solver_.rel_tol = rel_tol;
  auto status = linear_solver_.solve(dyn_A, rhs, state, rt);
  // TODO: handle failure.
}

}  // namespace silk::cuda
