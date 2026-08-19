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
#include "common/timer.hpp"

namespace silk::cuda {

namespace {

void update_aux_and_lagrange_mul(ObjRegistry& registry, float max_lagrange_mul,
                                 ctd::span<const float> state,
                                 ADMMResidualView global_residual,
                                 CudaRuntime rt) {
  auto clothes =
      registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache,
                                          ClothADMMHelper>();
  for (uint32_t e : clothes) {
    auto phy_state = registry.get<PhysicalState>(e);
    auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
    auto admm_helper = registry.get<ClothADMMHelper>(e);
    assert(phy_state && l1_cache && admm_helper);

    int off = phy_state->state_offset;
    int num = phy_state->state_num;
    auto sub_state = state.subspan(off, num);
    auto sub_dual = global_residual.dual_residual.subspan(off, num);
    auto sub_dual_scale_curr =
        global_residual.dual_scale_curr.subspan(off, num);
    auto sub_dual_scale_prev =
        global_residual.dual_scale_prev.subspan(off, num);

    ADMMResidualView residual{global_residual.primal_norm2,
                              global_residual.primal_scale_x2,
                              global_residual.primal_scale_aux2,
                              sub_dual,
                              sub_dual_scale_curr,
                              sub_dual_scale_prev};

    admm_helper->update_aux_var_and_lagrange_mul(max_lagrange_mul, *l1_cache,
                                                 sub_state, residual, rt);
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

    int offset = phy_state->state_offset;
    int state_num = phy_state->state_num;
    auto x = state.subspan(offset, state_num);
    auto local_lhs_diag = lhs_diag.subspan(offset, state_num);
    auto local_rhs = rhs.subspan(offset, state_num);
    auto local_inertia_mod = inertia_mod.subspan(offset, state_num);

    // We don't update collision in inner loop, so except the first solve skip
    // factorization.
    bool is_lhs_changed = (inner_iter == 1);
    admm_helper->solve_main_var(rel_tol, abs_tol, *l1_cache, is_lhs_changed,
                                local_lhs_diag, local_rhs, local_inertia_mod, x,
                                rt);
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

int compute_physical_primal_residual_dof(ObjRegistry& registry) {
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

/// @brief ADMM convergence tolerance.
///
/// ADMM residual has the form r = ||a-b||.
///
/// Absolute tolerance should scale with sqrt(r_dof).
/// Relative tolerance should scale with max(||a||, ||b||).
class StoppingCriteria {
 public:
  float eps = 0.0;

  StoppingCriteria(int dof, float a2, float b2, float abs_tol, float rel_tol) {
    float sqrt_dof = std::sqrt(dof);
    float a = std::sqrt(a2);
    float b = std::sqrt(b2);
    float max_scale = std::max(a, b);
    eps = sqrt_dof * abs_tol + max_scale * rel_tol;
  }

  bool has_converged(float residual) const { return (residual <= eps); }
};

}  // namespace

std::optional<ADMMSolver::Error> ADMMSolver::solve(
    ObjRegistry& registry, ctd::span<const float> prev_state,
    ctd::span<const float> prev_velocity, ctd::span<float> inner_state,
    EqualityConstraints& pin_constraints,
    ContactConstraints* contact_constraints, float dt, Vec3f const_acceleration,
    CudaRuntime rt) {
  int state_num = prev_state.size();
  if (state_num != cached_state_num_) {
    Timer timer_alloc_cache("alloc device cache storage");
    lhs_diag_ = alloc<float>(rt, state_num, 0);
    rhs_ = alloc<float>(rt, state_num, 0);
    inertia_mod_ = alloc<float>(rt, state_num);
    scalar_primal_norm2_ = alloc<float>(rt, 1);
    scalar_primal_scale_x2_ = alloc<float>(rt, 1);
    scalar_primal_scale_aux2_ = alloc<float>(rt, 1);
    dual_residual_ = alloc<float>(rt, state_num);
    dual_scale_curr_ = alloc<float>(rt, state_num);
    dual_scale_prev_ = alloc<float>(rt, state_num);
    scalar_dual_norm2_ = alloc<float>(rt, 1);
    scalar_dual_scale_curr_norm2_ = alloc<float>(rt, 1);
    scalar_dual_scale_prev_norm2_ = alloc<float>(rt, 1);
    cached_state_num_ = state_num;
  }

  Timer timer_compute_inertia_mod("compute intertia mod");
  int grid_num = div_round_up(state_num, 128);
  compute_inertia_mod<<<grid_num, 128, 0, rt.stream.get()>>>(
      dt, prev_state, prev_velocity, const_acceleration, *inertia_mod_);
  timer_compute_inertia_mod.end();

  pin_constraints.reset_lagrange_mul(rt);

  Timer timer_inner_loop("inner loop");
  float h_init_primal_norm = 0.0f;
  float h_init_dual_norm = 0.0f;
  float h_linear_tol_adaptive_ratio = 1.0f;
  for (int inner_it = 0; inner_it < max_inner_iteration; ++inner_it) {
    SPDLOG_DEBUG("Inner iter {}", inner_it);

    // 1. Optimize main variables
    // (except iter 0 since aux variables are not initialized).
    // 2. Optimize aux variables.
    // 3. Update lagrange multiplers.

    // Update main.
    if (inner_it != 0) {
      Timer timer_update_main("update main");

      cu::fill_bytes(rt.stream, *lhs_diag_, 0);
      cu::fill_bytes(rt.stream, *rhs_, 0);
      pin_constraints.eval(*lhs_diag_, *rhs_, rt);
      if (contact_constraints) {
        contact_constraints->eval(*lhs_diag_, *rhs_, rt);
      }

      // Linear solve tolerance is scaled by ADMM residual reduction ratio.
      // ratio = curr ADMM residual / init ADMM residual.
      float h_linear_rel_tol =
          ctd::clamp(initial_linear_rel_tol * h_linear_tol_adaptive_ratio,
                     linear_rel_tol_min, linear_rel_tol_max);
      SPDLOG_DEBUG("Linear solver tolerance rel={} abs={}", h_linear_rel_tol,
                   linear_abs_tol);

      update_main(registry, inner_it, h_linear_rel_tol, linear_abs_tol,
                  *lhs_diag_, *rhs_, *inertia_mod_, inner_state, rt);
    }

    // Update aux and lagrange multipliers.
    Timer timer_update_aux_and_mul("update aux var and multipliers");
    cu::fill_bytes(rt.stream, *scalar_primal_norm2_, 0);
    cu::fill_bytes(rt.stream, *scalar_primal_scale_x2_, 0);
    cu::fill_bytes(rt.stream, *scalar_primal_scale_aux2_, 0);
    cu::fill_bytes(rt.stream, *dual_residual_, 0);
    cu::fill_bytes(rt.stream, *dual_scale_curr_, 0);
    cu::fill_bytes(rt.stream, *dual_scale_prev_, 0);
    ADMMResidualView residual{
        scalar_primal_norm2_->data(),
        scalar_primal_scale_x2_->data(),
        scalar_primal_scale_aux2_->data(),
        *dual_residual_,
        *dual_scale_curr_,
        *dual_scale_prev_,
    };
    update_aux_and_lagrange_mul(registry, max_lagrange_mul, inner_state,
                                residual, rt);
    pin_constraints.update_lagrange_mul(inner_state, rt);
    pin_constraints.accum_primal_residual(inner_state, residual, rt);
    if (contact_constraints) {
      contact_constraints->update_aux_var_and_lagrange_mul(inner_state,
                                                           residual, rt);
    }
    timer_update_aux_and_mul.end();

    // Skip convergence check for iter 0.
    if (inner_it == 0) {
      continue;
    }

    Timer timer_convergence_check("convergence check");
    auto [min, max] = min_max(inner_state, rt);
    if (!(std::isfinite(min) && std::isfinite(max))) {
      SPDLOG_ERROR("solver explodes");
      return Error::Diverge;
    }

    // Compute primal residual and it's stopping criteria.
    float h_primal_norm =
        std::sqrt(scalar_load(scalar_primal_norm2_->data(), rt));
    if (inner_it == 1) {
      h_init_primal_norm = h_primal_norm;
    }
    float h_primal_scale_x2 = scalar_load(scalar_primal_scale_x2_->data(), rt);
    float h_primal_scale_aux2 =
        scalar_load(scalar_primal_scale_aux2_->data(), rt);
    int h_primal_residual_dof = compute_physical_primal_residual_dof(registry);
    h_primal_residual_dof += pin_constraints.get_active_dof();
    if (contact_constraints) {
      h_primal_residual_dof += contact_constraints->get_active_dof();
    }
    StoppingCriteria primal_criteria{h_primal_residual_dof, h_primal_scale_x2,
                                     h_primal_scale_aux2, non_linear_abs_tol,
                                     non_linear_rel_tol};

    // Compute dual residual and it's stopping criteria.
    inner_product(*dual_residual_, *dual_residual_, *scalar_dual_norm2_, rt);
    float h_dual_norm = std::sqrt(scalar_load(scalar_dual_norm2_->data(), rt));
    if (inner_it == 1) {
      h_init_dual_norm = h_dual_norm;
    }
    inner_product(*dual_scale_curr_, *dual_scale_curr_,
                  *scalar_dual_scale_curr_norm2_, rt);
    inner_product(*dual_scale_prev_, *dual_scale_prev_,
                  *scalar_dual_scale_prev_norm2_, rt);
    float h_dual_scale_curr2 =
        scalar_load(scalar_dual_scale_curr_norm2_->data(), rt);
    float h_dual_scale_prev2 =
        scalar_load(scalar_dual_scale_prev_norm2_->data(), rt);
    StoppingCriteria dual_criteria{state_num, h_dual_scale_curr2,
                                   h_dual_scale_prev2, non_linear_abs_tol,
                                   non_linear_rel_tol};

    // Update linear solver tolerance scaling.
    float h_primal_denom = std::max(h_init_primal_norm, non_linear_abs_tol);
    float h_dual_denom = std::max(h_init_dual_norm, non_linear_abs_tol);
    h_linear_tol_adaptive_ratio =
        std::max(h_primal_norm / h_primal_denom, h_dual_norm / h_dual_denom);

    SPDLOG_DEBUG("Primal norm {}. Criteria {}.", h_primal_norm,
                 primal_criteria.eps);
    SPDLOG_DEBUG("Dual norm {}. Criteria {}.", h_dual_norm, dual_criteria.eps);
    if (primal_criteria.has_converged(h_primal_norm) &&
        dual_criteria.has_converged(h_dual_norm)) {
      SPDLOG_INFO("ADMM it {}, residual [{}, {}], terminate.", inner_it,
                  h_primal_norm, h_dual_norm);
      break;
    }
    timer_convergence_check.end();
  }
  timer_inner_loop.end();

  // Small violation of constraints will cause later zero toi.
  Timer timer_enforce_constraints("enforce constraints");
  pin_constraints.enforce(inner_state, rt);
  if (contact_constraints) {
    contact_constraints->project(inner_state, rt);
  }
  timer_enforce_constraints.end();

  return std::nullopt;
}

}  // namespace silk::cuda
