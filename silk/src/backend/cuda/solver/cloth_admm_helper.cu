#include "backend/cuda/solver/cloth_admm_helper.cuh"

#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/cusparse_wrapper.hpp"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/svd.cuh"
#include "common/timer.hpp"

namespace silk::cuda {

namespace {

using Vec6f = Mat<float, 6, 1>;

__device__ Vec6f solve_elastic_aux(float penalty, float stiffness, float weight,
                                   const Vec6f& Sx, const Vec6f& u) {
  auto y = axpby(1.0, Sx, 1.0f / (penalty * weight), u);
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
  // Vectorize.
  Mat<float, 6, 1> proj_vec;
  proj_vec(0) = proj(0, 0);
  proj_vec(1) = proj(1, 0);
  proj_vec(2) = proj(2, 0);
  proj_vec(3) = proj(0, 1);
  proj_vec(4) = proj(1, 1);
  proj_vec(5) = proj(2, 1);
  float r = stiffness / (stiffness + penalty);
  Mat<float, 6, 1> aux = axpby(1 - r, y, r, proj_vec);

  return aux;
}

// clang-format off
__global__ void solve_bending_aux_and_prepare_dual(
    float max_u,
    float bending_stiffness,
    float penalty,
    ctd::span<const float> Sx,
    ctd::span<const float> w,
    ctd::span<const float> rest_curvature,
    ctd::span<float> u,
    ctd::span<float> z,
    ctd::span<float> weighted_delta,
    ctd::span<float> weighted_curr,
    ctd::span<float> weighted_prev,
    ctd::span<float> primal_norm2,
    ctd::span<float> primal_scale_x2,
    ctd::span<float> primal_scale_aux2)
// clang-format on
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float local_primal_norm2 = 0.0f;
  float local_primal_scale_x2 = 0.0f;
  float local_primal_scale_aux2 = 0.0f;

  if (tid < z.size()) {
    // Solve bending aux: z lies between y = Sx + u / (rho w) and rest
    // curvature p = Cx0.
    float old_z = z[tid];
    float weight = w[tid];
    float y = Sx[tid] + u[tid] / (penalty * weight);
    float r = bending_stiffness / (bending_stiffness + penalty);
    float new_z = (1.0f - r) * y + r * rest_curvature[tid];
    z[tid] = new_z;

    // Primal residual: W(Sx - z).
    float primal = weight * (Sx[tid] - new_z);
    local_primal_norm2 = primal * primal;
    float wSx = weight * Sx[tid];
    local_primal_scale_x2 = wSx * wSx;
    float wz = weight * new_z;
    local_primal_scale_aux2 = wz * wz;

    // Lagrange multiplier update: u += rho W(Sx - z).
    float delta = penalty * primal;
    u[tid] = ctd::clamp(u[tid] + delta, -max_u, max_u);

    // Precompute rho W^T W z terms; the caller applies S^T.
    float dual_curr = penalty * weight * weight * new_z;
    float dual_prev = penalty * weight * weight * old_z;
    weighted_delta[tid] = dual_curr - dual_prev;
    weighted_curr[tid] = dual_curr;
    weighted_prev[tid] = dual_prev;
  }

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

__global__ void prepare_bending_rhs_vec(float penalty, ctd::span<const float> w,
                                        ctd::span<const float> u,
                                        ctd::span<const float> z,
                                        ctd::span<float> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= out.size()) {
    return;
  }
  // RHS contribution before S^T: -W u + rho W^T W z.
  out[tid] = -w[tid] * u[tid] + penalty * w[tid] * w[tid] * z[tid];
}

// @brief Solve elastic aux and its multipliers.
//
// ADMM objective:
// L(x,z) = f(x) + g(z) + u||W(Sx-z)|| + ½ rho||W(Sx-z)||^2
// g(z) = min_p ( ½ k||W(z-p)||^2 )
//
// x = Main variables, z = Elastic aux variables.
// f(x) = Main function, typically momentum term.
// g(z) = Elastic energy. Projective dynamic style.
// u = Lagrange multipliers, rho = Penalty
// k = Stiffness, W = Weight, p = Projection.
//
// In projective dynamic, z is the deformation gradient F and p is the
// projection of F on rest manifold. a.k.a. Nearest F where deformation is zero.
// Practically we achieve this by doing SVD then clamp diagonal between 0.9
// and 1.1 as cloth is generallly not stretchable. The energy is simply the
// distance between F and rest F. Naturally, S here is the jacobian operator and
// weight w is triangle area sqrt.
//
// To solve z, we observe that
// ∂L/∂z=0 -> argmin_z min_p ( ½ k||w(z-p)||^2 + ½ rho||w(Sx-z+u/(wp))||^2 )
// Let y = Sx+u/(wp), intuitively, z must lies on line y <-> p where p =
// proj(y). So we first solve p then compute z based on the ratio between k and
// rho.
//
// See paper 10.5555/2982818.2982822 for detail. The formulation is slightly
// different as I use unweighted u.
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
    ctd::span<float> dual_residual,
    ctd::span<float> dual_scale_curr,
    ctd::span<float> dual_scale_prev)
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
    Mat<float, 6, 1> old_z;
    int aux_var_offset = tid * 6;
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      old_z(i) = aux_var[aux_var_offset + i];
    }
    // Gather jacobian op.
    Mat<float, 6, 9> jop;
    int jop_offset = 54 * tid;
    for (int i = 0; i < 54; ++i) {
      jop.data[i] = jacobian_ops[jop_offset + i];
    }
    // Gather weight.
    float w = area_sqrt[tid];

    // Compute aux variable.
    Vec6f Sx = mat_mul(jop, x);
    Vec6f z = solve_elastic_aux(penalty, elastic_stiffness, w, Sx, u);

    // Per face primal residual.
    // r = W(Sx - z)
    auto primal = ax(w, vsub(Sx, z));
    local_primal_norm2 = squared_norm(primal);
    local_primal_scale_x2 = squared_norm(ax(w, Sx));
    local_primal_scale_aux2 = squared_norm(ax(w, z));

    // Per state dof dual residual.
    // r = rho * S^T W^T W (z - z_old)
    auto S_tr = jop.view().transpose();
    auto dual_curr = ax(penalty * w * w, mat_mul(S_tr, z));
    auto dual_prev = ax(penalty * w * w, mat_mul(S_tr, old_z));
    auto dual = vsub(dual_curr, dual_prev);
    for (int i = 0; i < 3; ++i) {
      int out_offset = 3 * faces[tid * 3 + i];
#pragma unroll
      for (int j = 0; j < 3; ++j) {
        atomicAdd(dual_residual.data() + out_offset + j, dual(3 * i + j));
        atomicAdd(dual_scale_curr.data() + out_offset + j,
                  dual_curr(3 * i + j));
        atomicAdd(dual_scale_prev.data() + out_offset + j,
                  dual_prev(3 * i + j));
      }
    }

    // Write aux var.
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      aux_var[aux_var_offset + i] = z(i);
    }

    // Update lagrange multipliers.
    // u += rho W (Sx-z).
    auto delta = axpby(w * penalty, Sx, -w * penalty, z);
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      u(i) = ctd::clamp(u(i) + delta(i), -max_lagrange_mul, max_lagrange_mul);
    }

    int mul_offset = tid * 6;
#pragma unroll
    for (int i = 0; i < 6; ++i) {
      lagrange_mul[mul_offset + i] = u(i);
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

// y += Ax
void Axpy(BSRView A, ctd::span<const float> x, ctd::span<float> y,
          const CuSparseHandle& cusparse_handle,
          cu::device_buffer<char>& cusparse_workspace, CudaRuntime rt) {
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_ops{A};
  CuSparseConstVec cusparse_in{x};
  CuSparseVec cusparse_out{y};

  float alpha = 1.0f;
  float beta = 1.0f;
  size_t workspace_size;
  cusparseSpMV_bufferSize(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE,
                          &alpha, cusparse_ops.raw, cusparse_in.raw, &beta,
                          cusparse_out.raw, CUDA_R_32F,
                          CUSPARSE_SPMV_ALG_DEFAULT, &workspace_size);
  if (cusparse_workspace.size() < workspace_size) {
    cusparse_workspace = alloc<char>(rt, workspace_size);
  }
  cusparseSpMV(cusparse_handle.raw, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
               cusparse_ops.raw, cusparse_in.raw, &beta, cusparse_out.raw,
               CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT,
               cusparse_workspace.data());
}

// @brief Solve bending aux and its multipliers.
//
// ADMM objective:
// L(x,z) = f(x) + g(z) + u||W(Sx-z)|| + ½ rho||W(Sx-z)||^2
// g(z) = min_p ( ½ k||W(z-p)||^2 )
//
// x = Main variables, z = Bending aux variables.
// f(x) = Main function, typically momentum term.
// g(z) = Bending energy. Projective dynamic style.
// u = Lagrange multipliers, rho = Penalty
// k = Stiffness, W = Weight, p = Projection.
//
// In projective dynamic, z is the curvature and p is the projection of
// curvature on rest manifold. a.k.a. Rest curvature from initial position.
// The energy is simply the distance between current curvature and rest
// curvature. Under the assumption that cloth barely stretches, S is the
// cotangent matrix from rest position and weight w is inverse voroni mass sqrt.
//
// To solve z, we observe that
// ∂L/∂z=0 -> argmin_z min_p ( ½ k||w(z-p)||^2 + ½ rho||w(Sx-z+u/(wp))||^2 )
// Let y = Sx+u/(wp), intuitively, z must lies on line y <-> p where p =
// proj(y). So we first solve p then compute z based on the ratio between k and
// rho.
//
// See paper 10.5555/2982818.2982822 for detail. The formulation is slightly
// different as I use unweighted u.
// clang-format off
void solve_and_update_bending_aux(
    float max_lagrange_mul,
    float bending_stiffness,
    float penalty,
    ctd::span<const float> position,
    BSRView laplacian_ops,
    ctd::span<const float> bending_weight_sqrt,
    ctd::span<const float> rest_curvature,
    const CuSparseHandle& cusparse_handle,
    cu::device_buffer<char>& cusparse_workspace,
    cu::device_buffer<float>& Sx_tmp,
    cu::device_buffer<float>& weighted_curr_tmp,
    cu::device_buffer<float>& weighted_prev_tmp,
    ctd::span<float> lagrange_mul,
    ctd::span<float> aux_var,
    ctd::span<float> primal_norm2,
    ctd::span<float> primal_scale_x2,
    ctd::span<float> primal_scale_aux2,
    ctd::span<float> dual_residual,
    ctd::span<float> dual_scale_curr,
    ctd::span<float> dual_scale_prev,
    CudaRuntime rt)
// clang-format on
{
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());
  CuSparseBSR cusparse_lap{laplacian_ops};
  CuSparseConstVec cusparse_x{position};

  if (Sx_tmp.size() < position.size()) {
    Sx_tmp = alloc<float>(rt, position.size());
  }
  if (weighted_curr_tmp.size() < position.size()) {
    weighted_curr_tmp = alloc<float>(rt, position.size());
  }
  if (weighted_prev_tmp.size() < position.size()) {
    weighted_prev_tmp = alloc<float>(rt, position.size());
  }
  CuSparseVec cusparse_Sx{Sx_tmp};

  // Compute curvature Sx = Cx.
  float alpha = 1.0f;
  float beta = 0.0f;
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

  // Solve z, update u, and prepare rho W^T W z terms.
  int grid_num = div_round_up(lagrange_mul.size(), 128);
  solve_bending_aux_and_prepare_dual<<<grid_num, 128, 0, rt.stream.get()>>>(
      max_lagrange_mul, bending_stiffness, penalty, Sx_tmp, bending_weight_sqrt,
      rest_curvature, lagrange_mul, aux_var, Sx_tmp, weighted_curr_tmp,
      weighted_prev_tmp, primal_norm2, primal_scale_x2, primal_scale_aux2);

  // Compute dual residual: S^T rho W^T W (z_curr - z_prev).
  ctd::span<const float> weighted_delta{Sx_tmp};
  Axpy(laplacian_ops, weighted_delta, dual_residual, cusparse_handle,
       cusparse_workspace, rt);
  Axpy(laplacian_ops, weighted_curr_tmp, dual_scale_curr, cusparse_handle,
       cusparse_workspace, rt);
  Axpy(laplacian_ops, weighted_prev_tmp, dual_scale_prev, cusparse_handle,
       cusparse_workspace, rt);
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

void assemble_bending_rhs(float penalty, BSRView laplacian_ops,
                          ctd::span<const float> bending_weight_sqrt,
                          ctd::span<const float> lagrange_mul,
                          ctd::span<const float> aux_var,
                          const CuSparseHandle& cusparse_handle,
                          cu::device_buffer<char>& cusparse_workspace,
                          cu::device_buffer<float>& tmp, ctd::span<float> rhs,
                          CudaRuntime rt) {
  cusparseSetStream(cusparse_handle.raw, rt.stream.get());

  if (tmp.size() < rhs.size()) {
    tmp = alloc<float>(rt, rhs.size());
  }
  int grid_num = div_round_up(tmp.size(), 128);
  prepare_bending_rhs_vec<<<grid_num, 128, 0, rt.stream.get()>>>(
      penalty, bending_weight_sqrt, lagrange_mul, aux_var, tmp);

  Axpy(laplacian_ops, tmp, rhs, cusparse_handle, cusparse_workspace, rt);
}

}  // namespace

ClothADMMHelper::ClothADMMHelper(int vert_num, int face_num, CudaRuntime rt) {
  int jacobian_dof = 6 * face_num;
  ze_ = alloc<float>(rt, jacobian_dof);
  ue_ = alloc<float>(rt, jacobian_dof, 0);
  // Curvature is per vertex coordinate.
  int curvature_dof = 3 * vert_num;
  zb_ = alloc<float>(rt, curvature_dof);
  ub_ = alloc<float>(rt, curvature_dof, 0);

  cusparse_workspace_ = alloc<char>(rt, 0);
  float_tmp_ = alloc<float>(rt, 0);
  float_tmp2_ = alloc<float>(rt, 0);
  float_tmp3_ = alloc<float>(rt, 0);
}

void ClothADMMHelper::reset_aux_lagrange_mul(CudaRuntime rt) {
  cu::fill_bytes(rt.stream, *ze_, 0);
  cu::fill_bytes(rt.stream, *ue_, 0);
  cu::fill_bytes(rt.stream, *zb_, 0);
  cu::fill_bytes(rt.stream, *ub_, 0);
}

void ClothADMMHelper::update_aux_var_and_lagrange_mul(
    float max_lagrange_mul, const ClothAssemblyL1Cache& l1_cache,
    ctd::span<const float> state, ctd::span<float> primal_residual_norm2,
    ctd::span<float> primal_scale_x2, ctd::span<float> primal_scale_aux2,
    ctd::span<float> dual_residual, ctd::span<float> dual_scale_curr,
    ctd::span<float> dual_scale_prev, CudaRuntime rt) {
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
        *ue_,
        *ze_,
        primal_residual_norm2,
        primal_scale_x2,
        primal_scale_aux2,
        dual_residual,
        dual_scale_curr,
        dual_scale_prev);

  solve_and_update_bending_aux(
        max_lagrange_mul,
        l1_cache.bending_stiffness,
        l1_cache.bending_stiffness,
        state,
        l1_cache.laplacian_ops.view(),
        *l1_cache.bending_weight_sqrt,
        *l1_cache.rest_curvature,
        cusparse_handle_,
        *cusparse_workspace_,
        *float_tmp_,
        *float_tmp2_,
        *float_tmp3_,
        *ub_,
        *zb_,
        primal_residual_norm2,
        primal_scale_x2,
        primal_scale_aux2,
        dual_residual,
        dual_scale_curr,
        dual_scale_prev,
        rt);
  // clang-format on
}

void ClothADMMHelper::solve_main_var(float rel_tol, float abs_tol,
                                     const ClothAssemblyL1Cache& l1_cache,
                                     bool is_lhs_changed,
                                     ctd::span<const float> extern_lhs,
                                     ctd::span<const float> extern_rhs,
                                     ctd::span<const float> inertia_mod,
                                     ctd::span<float> state, CudaRuntime rt) {
  Timer timer_copy_cached("copy cached lhs and rhs");
  auto lhs_diag = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_lhs, lhs_diag);
  auto rhs = alloc<float>(rt, l1_cache.state_num);
  cu::copy_bytes(rt.stream, extern_rhs, rhs);
  timer_copy_cached.end();

  Timer timer_assemble_inertia("assemble inertia");
  int grid_num = div_round_up(l1_cache.state_num, 128);
  assemble_inertia<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.dt, *l1_cache.mass, state, inertia_mod, lhs_diag, rhs);
  timer_assemble_inertia.end();

  Timer timer_assemble_elastic_rhs("assemble elastic rhs");
  grid_num = div_round_up(l1_cache.face_num, 128);
  assemble_elastic_rhs<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.face_num, l1_cache.penalty, *l1_cache.faces,
      *l1_cache.jacobian_ops, *l1_cache.area_sqrt, *ue_, *ze_, rhs);
  timer_assemble_elastic_rhs.end();

  Timer timer_assemble_bending_rhs("assemble bending rhs");
  if (!float_tmp_) {
    float_tmp_ = alloc<float>(rt, rhs.size());
  }
  if (l1_cache.bending_stiffness > 0.0f) {
    assemble_bending_rhs(
        l1_cache.bending_stiffness, l1_cache.laplacian_ops.view(),
        *l1_cache.bending_weight_sqrt, *ub_, *zb_, cusparse_handle_,
        *cusparse_workspace_, *float_tmp_, rhs, rt);
  }
  timer_assemble_bending_rhs.end();

  DynamicBSRView dyn_A{lhs_diag, l1_cache.weighted_AA.view()};
  Timer timer_mas_factorize("MAS solver factorize");
  if (is_lhs_changed) {
    linear_solver_.factorize(dyn_A, *l1_cache.part_offsets, rt);
  }
  timer_mas_factorize.end();

  linear_solver_.abs_tol = abs_tol;
  linear_solver_.rel_tol = rel_tol;
  Timer timer_mas_solve("MAS solve");
  auto status = linear_solver_.solve(dyn_A, rhs, state, rt);
  timer_mas_solve.end();
  // TODO: handle failure.
}

}  // namespace silk::cuda
