#include "backend/cuda/solver/cloth_admm_helper.cuh"

#include <cub/cub.cuh>
#include <cuda/algorithm>
#include <cuda/std/span>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
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
    ADMMResidualView residual)
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
        atomicAdd(residual.dual_residual.data() + out_offset + j,
                  dual(3 * i + j));
        atomicAdd(residual.dual_scale_curr.data() + out_offset + j,
                  dual_curr(3 * i + j));
        atomicAdd(residual.dual_scale_prev.data() + out_offset + j,
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
    atomicAdd(residual.primal_norm2, reduced);
  }
  __syncthreads();
  reduced = BlockReduce(reduce_tmp).Sum(local_primal_scale_x2);
  if (threadIdx.x == 0) {
    atomicAdd(residual.primal_scale_x2, reduced);
  }
  __syncthreads();
  reduced = BlockReduce(reduce_tmp).Sum(local_primal_scale_aux2);
  if (threadIdx.x == 0) {
    atomicAdd(residual.primal_scale_aux2, reduced);
  }
}

__global__ void assemble_inertia_and_bending(float dt,
                                             ctd::span<const float> mass,
                                             ctd::span<const float> inertia_mod,
                                             ctd::span<const float> bending_rhs,
                                             ctd::span<float> lhs_diag,
                                             ctd::span<float> rhs) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= lhs_diag.size()) {
    return;
  }

  lhs_diag[tid] += mass[tid] / (dt * dt);
  rhs[tid] += bending_rhs[tid] - mass[tid] * inertia_mod[tid];
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

}  // namespace

ClothADMMHelper::ClothADMMHelper(int face_num, CudaRuntime rt) {
  int jacobian_dof = 6 * face_num;
  ze_ = alloc<float>(rt, jacobian_dof);
  ue_ = alloc<float>(rt, jacobian_dof, 0);
}

void ClothADMMHelper::reset_aux_lagrange_mul(CudaRuntime rt) {
  cu::fill_bytes(rt.stream, *ze_, 0);
  cu::fill_bytes(rt.stream, *ue_, 0);
}

void ClothADMMHelper::update_aux_var_and_lagrange_mul(
    float max_lagrange_mul, const ClothAssemblyL1Cache& l1_cache,
    ctd::span<const float> state, ADMMResidualView residual, CudaRuntime rt) {
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
        residual);
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

  Timer timer_assemble_inertia_bending("assemble inertia and bending");
  int grid_num = div_round_up(l1_cache.state_num, 128);
  assemble_inertia_and_bending<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.dt, *l1_cache.mass, inertia_mod, *l1_cache.bending_rhs, lhs_diag,
      rhs);
  timer_assemble_inertia_bending.end();

  Timer timer_assemble_elastic_rhs("assemble elastic rhs");
  grid_num = div_round_up(l1_cache.face_num, 128);
  assemble_elastic_rhs<<<grid_num, 128, 0, rt.stream.get()>>>(
      l1_cache.face_num, l1_cache.penalty, *l1_cache.faces,
      *l1_cache.jacobian_ops, *l1_cache.area_sqrt, *ue_, *ze_, rhs);
  timer_assemble_elastic_rhs.end();

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
