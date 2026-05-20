#include "backend/cuda/solver/admm_solver.cuh"

#include <algorithm>
#include <cmath>
#include <cub/cub.cuh>
#include <cuda/atomic>
#include <cuda/functional>
#include <cuda/std/limits>

#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/simple_linalg.cuh"
#include "backend/cuda/solver/cloth_admm_helper.cuh"
#include "backend/cuda/solver/inner_product.cuh"
#include "common/logger.hpp"

namespace silk::cuda {

namespace {

void update_aux_and_lagrange_mul(
    ObjRegistry& registry, float max_lagrange_mul, ctd::span<const float> state,
    ctd::span<float> primal_norm2, ctd::span<float> primal_scale_x2,
    ctd::span<float> primal_scale_aux2, ctd::span<float> dual_residual,
    ctd::span<float> dual_scale_curr, ctd::span<float> dual_scale_prev,
    CudaRuntime rt) {
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<ClothADMMHelper>(e);
    assert(phy_state && l1_cache && admm_helper);

    auto sub_state =
        state.subspan(phy_state->state_offset, phy_state->state_num);
    auto sub_dual =
        dual_residual.subspan(phy_state->state_offset, phy_state->state_num);
    auto sub_dual_scale_curr =
        dual_scale_curr.subspan(phy_state->state_offset, phy_state->state_num);
    auto sub_dual_scale_prev =
        dual_scale_prev.subspan(phy_state->state_offset, phy_state->state_num);
    admm_helper->update_aux_var_and_lagrange_mul(
        max_lagrange_mul, *l1_cache, sub_state, primal_norm2, primal_scale_x2,
        primal_scale_aux2, sub_dual, sub_dual_scale_curr, sub_dual_scale_prev,
        rt);
  }
}

void update_main(ObjRegistry& registry, int inner_iter, float rel_tol,
                 float abs_tol, ctd::span<const float> lhs_diag,
                 ctd::span<const float> rhs, ctd::span<const float> inertia_mod,
                 ctd::span<float> state, CudaRuntime rt) {
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<ClothADMMHelper>(e);
    assert(phy_state && l1_cache && admm_helper);

    auto x = state.subspan(phy_state->state_offset, phy_state->state_num);
    bool is_lhs_changed = (inner_iter == 1);
    admm_helper->solve_main_var(rel_tol, abs_tol, *l1_cache, is_lhs_changed,
                                lhs_diag, rhs, inertia_mod, x, rt);
  }
}

__global__ void compute_inertia_mod(float dt, ctd::span<const float> state,
                                    ctd::span<const float> velocity, Vec3f acc,
                                    ctd::span<float> out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= state.size()) {
    return;
  }

  float a = acc(tid % 3);
  out[tid] = -state[tid] / (dt * dt) - velocity[tid] / dt - a;
}

__global__ void min_max_kernel(ctd::span<const float> values, float* min_out,
                               float* max_out) {
  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  float min = ctd::numeric_limits<float>::infinity();
  float max = -ctd::numeric_limits<float>::infinity();
  if (tid < values.size()) {
    min = values[tid];
    max = values[tid];
  }

  float block_min = BlockReduce(tmp).Reduce(min, cu::minimum<float>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> amin{*min_out};
    amin.fetch_min(block_min);
  }
  __syncthreads();

  float block_max = BlockReduce(tmp).Reduce(max, cu::maximum<float>{});
  if (threadIdx.x == 0) {
    cu::atomic_ref<float> amax{*max_out};
    amax.fetch_max(block_max);
  }
}

std::pair<float, float> min_max(ctd::span<const float> values, CudaRuntime rt) {
  auto d_min = alloc<float>(rt, 1, ctd::numeric_limits<float>::infinity());
  auto d_max = alloc<float>(rt, 1, -ctd::numeric_limits<float>::infinity());

  int grid_num = div_round_up(values.size(), 128);
  min_max_kernel<<<grid_num, 128, 0, rt.stream.get()>>>(values, d_min.data(),
                                                        d_max.data());
  float h_min = scalar_load(d_min.data(), rt);
  float h_max = scalar_load(d_max.data(), rt);
  return std::make_pair(h_min, h_max);
}

}  // namespace

namespace {

int primal_residual_dof(ObjRegistry& registry) {
  int dof = 0;
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    assert(l1_cache);
    dof += 6 * l1_cache->face_num;
  }
  return dof;
}

}  // namespace

std::optional<ADMMSolver::Error> ADMMSolver::solve(
    ObjRegistry& registry, ctd::span<const float> prev_state,
    ctd::span<const float> prev_velocity, ctd::span<float> inner_state,
    EqualityConstraints& pin_constraints,
    EqualityConstraints* barrier_constraints, float dt,
    Vec3f const_acceleration, CudaRuntime rt) {
  int state_num = prev_state.size();
  int primal_dof = primal_residual_dof(registry);
  if (state_num != cached_state_num_) {
    inner_tmp_ = alloc<float>(rt, state_num);
    lhs_diag_ = alloc<float>(rt, state_num, 0);
    rhs_ = alloc<float>(rt, state_num, 0);
    inertia_mod_ = alloc<float>(rt, state_num);
    scalar_primal_norm2_ = alloc<float>(rt, 1);
    scalar_primal_scale_x2_ = alloc<float>(rt, 1);
    scalar_primal_scale_aux2_ = alloc<float>(rt, 1);
    scalar_equality_primal_dof_ = alloc<float>(rt, 1);
    dual_residual_ = alloc<float>(rt, state_num);
    dual_scale_curr_ = alloc<float>(rt, state_num);
    dual_scale_prev_ = alloc<float>(rt, state_num);
    scalar_dual_norm2_ = alloc<float>(rt, 1);
    scalar_dual_scale_curr_norm2_ = alloc<float>(rt, 1);
    scalar_dual_scale_prev_norm2_ = alloc<float>(rt, 1);
    cached_state_num_ = state_num;
  }

  int grid_num = div_round_up(state_num, 128);
  compute_inertia_mod<<<grid_num, 128, 0, rt.stream.get()>>>(
      dt, prev_state, prev_velocity, const_acceleration, *inertia_mod_);

  pin_constraints.reset_lagrange_mul(rt);
  if (barrier_constraints) {
    barrier_constraints->reset_lagrange_mul(rt);
  }

  float h_init_primal_norm = 0.0f;
  float h_init_dual_norm = 0.0f;
  float h_adaptive_ratio = 1.0f;
  cu::copy_bytes(rt.stream, inner_state, *inner_tmp_);
  for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
    SPDLOG_INFO("Inner iter {}", inner_it);

    if (inner_it != 0) {
      cu::fill_bytes(rt.stream, *lhs_diag_, 0);
      cu::fill_bytes(rt.stream, *rhs_, 0);
      pin_constraints.eval(*lhs_diag_, *rhs_, rt);
      if (barrier_constraints) {
        barrier_constraints->eval(*lhs_diag_, *rhs_, rt);
      }
      float h_linear_rel_tol = linear_rel_tol_max;
      if (inner_it > 1) {
        h_linear_rel_tol = ctd::clamp(linear_adaptive_factor * h_adaptive_ratio,
                                      linear_rel_tol_min, linear_rel_tol_max);
      }
      SPDLOG_INFO("Linear solver tolerance rel={} abs={}", h_linear_rel_tol,
                  linear_abs_tol);
      update_main(registry, inner_it, h_linear_rel_tol, linear_abs_tol,
                  *lhs_diag_, *rhs_, *inertia_mod_, *inner_tmp_, rt);
    }

    cu::fill_bytes(rt.stream, *scalar_primal_norm2_, 0);
    cu::fill_bytes(rt.stream, *scalar_primal_scale_x2_, 0);
    cu::fill_bytes(rt.stream, *scalar_primal_scale_aux2_, 0);
    cu::fill_bytes(rt.stream, *scalar_equality_primal_dof_, 0);
    cu::fill_bytes(rt.stream, *dual_residual_, 0);
    cu::fill_bytes(rt.stream, *dual_scale_curr_, 0);
    cu::fill_bytes(rt.stream, *dual_scale_prev_, 0);
    update_aux_and_lagrange_mul(registry, max_lagrange_mul, *inner_tmp_,
                                *scalar_primal_norm2_, *scalar_primal_scale_x2_,
                                *scalar_primal_scale_aux2_, *dual_residual_,
                                *dual_scale_curr_, *dual_scale_prev_, rt);
    pin_constraints.update_lagrange_mul(*inner_tmp_, rt);
    pin_constraints.accum_primal_residual(
        *inner_tmp_, *scalar_primal_norm2_, *scalar_primal_scale_x2_,
        *scalar_primal_scale_aux2_, *scalar_equality_primal_dof_, rt);
    if (barrier_constraints) {
      barrier_constraints->update_lagrange_mul(*inner_tmp_, rt);
      barrier_constraints->accum_primal_residual(
          *inner_tmp_, *scalar_primal_norm2_, *scalar_primal_scale_x2_,
          *scalar_primal_scale_aux2_, *scalar_equality_primal_dof_, rt);
    }

    if (inner_it == 0) {
      continue;
    }

    auto [min, max] = min_max(*inner_tmp_, rt);
    if (!(std::isfinite(min) && std::isfinite(max))) {
      SPDLOG_ERROR("solver explodes");
      return Error::Diverge;
    }

    float h_primal_norm2 = scalar_load(scalar_primal_norm2_->data(), rt);
    float h_primal_scale_x2 = scalar_load(scalar_primal_scale_x2_->data(), rt);
    float h_primal_scale_aux2 =
        scalar_load(scalar_primal_scale_aux2_->data(), rt);
    float h_equality_primal_dof =
        scalar_load(scalar_equality_primal_dof_->data(), rt);
    float h_primal_norm = std::sqrt(h_primal_norm2);
    float h_primal_scale_x = std::sqrt(h_primal_scale_x2);
    float h_primal_scale_aux = std::sqrt(h_primal_scale_aux2);
    inner_product(*dual_residual_, *dual_residual_, *scalar_dual_norm2_, rt);
    inner_product(*dual_scale_curr_, *dual_scale_curr_,
                  *scalar_dual_scale_curr_norm2_, rt);
    inner_product(*dual_scale_prev_, *dual_scale_prev_,
                  *scalar_dual_scale_prev_norm2_, rt);
    float h_dual_norm = std::sqrt(scalar_load(scalar_dual_norm2_->data(), rt));
    float h_dual_scale_curr =
        std::sqrt(scalar_load(scalar_dual_scale_curr_norm2_->data(), rt));
    float h_dual_scale_prev =
        std::sqrt(scalar_load(scalar_dual_scale_prev_norm2_->data(), rt));

    if (inner_it == 1) {
      h_init_primal_norm = h_primal_norm;
      h_init_dual_norm = h_dual_norm;
    }
    float h_primal_dof_total = primal_dof + h_equality_primal_dof;
    float h_state_dof = state_num;
    float h_primal_eps =
        std::sqrt(h_primal_dof_total) * non_linear_abs_tol +
        non_linear_rel_tol * std::max(h_primal_scale_x, h_primal_scale_aux);
    float h_dual_scale = std::max(h_dual_scale_curr, h_dual_scale_prev);
    float h_dual_eps = std::sqrt(h_state_dof) * non_linear_abs_tol +
                       non_linear_rel_tol * h_dual_scale;
    bool primal_converged = h_primal_norm <= h_primal_eps;
    bool dual_converged = h_dual_norm <= h_dual_eps;
    float h_primal_denom = std::max(h_init_primal_norm, non_linear_abs_tol);
    float h_dual_denom = std::max(h_init_dual_norm, non_linear_abs_tol);
    h_adaptive_ratio =
        std::max(h_primal_norm / h_primal_denom, h_dual_norm / h_dual_denom);
    SPDLOG_INFO("Primal norm {}. Criteria {}. Abs tol {}.", h_primal_norm,
                h_primal_eps, non_linear_abs_tol);
    SPDLOG_INFO("Dual norm {}. Criteria {}. Abs tol {}. Scale curr {} prev {}.",
                h_dual_norm, h_dual_eps, non_linear_abs_tol, h_dual_scale_curr,
                h_dual_scale_prev);
    if (primal_converged && dual_converged) {
      SPDLOG_INFO("ADMM residual [{}, {}], NL loop terminate", h_primal_norm,
                  h_dual_norm);
      cu::copy_bytes(rt.stream, *inner_tmp_, inner_state);
      break;
    }

    cu::copy_bytes(rt.stream, *inner_tmp_, inner_state);
  }

  pin_constraints.enforce(inner_state, rt);
  if (barrier_constraints) {
    barrier_constraints->enforce(inner_state, rt);
  }

  return std::nullopt;
}

}  // namespace silk::cuda
