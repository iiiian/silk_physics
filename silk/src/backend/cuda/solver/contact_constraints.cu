#include "backend/cuda/solver/contact_constraints.cuh"

#include <algorithm>
#include <cstdint>
#include <cub/cub.cuh>
#include <cuda/functional>
#include <cuda/std/cmath>
#include <tuple>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/solver/contact_coloring.cuh"
#include "common/logger.hpp"
#include "common/timer.hpp"

namespace silk::cuda {

namespace {

ContactKey make_contact_key(const Collision& collision) {
  ContactKey key{.type = collision.type,
                 .object_id_a = collision.object_id_a,
                 .object_id_b = collision.object_id_b,
                 .index = {collision.index(0), collision.index(1),
                           collision.index(2), collision.index(3)}};
  if (collision.type == CollisionType::PointTriangle) {
    std::sort(key.index.begin() + 1, key.index.end());
    return key;
  }

  if (key.index[1] < key.index[0]) {
    std::swap(key.index[0], key.index[1]);
  }
  if (key.index[3] < key.index[2]) {
    std::swap(key.index[2], key.index[3]);
  }
  auto edge_a = std::tie(key.object_id_a, key.index[0], key.index[1]);
  auto edge_b = std::tie(key.object_id_b, key.index[2], key.index[3]);
  if (edge_b < edge_a) {
    std::swap(key.object_id_a, key.object_id_b);
    std::swap(key.index[0], key.index[2]);
    std::swap(key.index[1], key.index[3]);
  }
  return key;
}

__device__ Vec3f collision_position(const Collision& collision, int slot) {
  if (slot == 0) {
    return collision.x0;
  }
  if (slot == 1) {
    return collision.x1;
  }
  if (slot == 2) {
    return collision.x2;
  }
  return collision.x3;
}

__device__ int collision_state_offset(const Collision& collision, int slot) {
  int state_offset;
  if (collision.type == CollisionType::PointTriangle) {
    state_offset =
        slot == 0 ? collision.state_offset_a : collision.state_offset_b;
  } else {
    state_offset =
        slot < 2 ? collision.state_offset_a : collision.state_offset_b;
  }
  return state_offset < 0 ? -1 : state_offset + 3 * collision.index(slot);
}

__global__ void build_contacts(ctd::span<const Collision> collisions,
                               ctd::span<const float> weight,
                               ctd::span<FrictionContact> contacts) {
  // Precompute the weighted minimum-distance scatter:
  //   u_c = Σⱼ b_c,j pⱼ + k_c
  //   s_c = Σⱼ b_c,j² / wⱼ
  //   γ_c,j = b_c,j / (s_c wⱼ)
  int contact_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (contact_index >= collisions.size()) {
    return;
  }

  const Collision& collision = collisions[contact_index];
  FrictionContact contact;
  contact.coefficient = collision.coefficient;
  contact.normal = collision.normal;
  contact.friction = collision.friction;
  contact.affine_offset = ax(-collision.activation_distance, collision.normal);

  float scatter_normalization = 0.0f;
  Vec3f reference_relative_position = Vec3f::zeros();
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    float coefficient = contact.coefficient(slot);
    reference_relative_position =
        vadd(reference_relative_position,
             ax(coefficient, collision_position(collision, slot)));

    int state_index = collision_state_offset(collision, slot);
    contact.state_offset(slot) = state_index;
    if (state_index < 0) {
      // Fold the obstacle reference position into the affine term k_c.
      contact.affine_offset =
          vadd(contact.affine_offset,
               ax(coefficient, collision_position(collision, slot)));
      continue;
    }
    if (collision.inv_mass(slot) > 0.0f) {
      scatter_normalization += coefficient * coefficient / weight[state_index];
    }
  }

  // Daviet's contact variable combines the target normal gap with relative
  // tangential displacement. Add prescribed obstacle motion, then remove the
  // reference tangent without changing the gap test.
  contact.affine_offset =
      vadd(contact.affine_offset, collision.prescribed_displacement);
  float reference_normal = dot(reference_relative_position, contact.normal);
  Vec3f reference_tangent =
      vsub(reference_relative_position, ax(reference_normal, contact.normal));
  contact.affine_offset = vsub(contact.affine_offset, reference_tangent);

  assert(scatter_normalization > 0.0f);
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    int state_index = contact.state_offset(slot);
    if (state_index >= 0 && collision.inv_mass(slot) > 0.0f) {
      contact.gamma(slot) = contact.coefficient(slot) /
                            (scatter_normalization * weight[state_index]);
    } else {
      contact.gamma(slot) = 0.0f;
    }
  }
  contacts[contact_index] = contact;
}

__global__ void count_vertex_incidence(
    ctd::span<const FrictionContact> contacts, ctd::span<int> counts) {
  int contact_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (contact_index >= contacts.size()) {
    return;
  }
  const FrictionContact& contact = contacts[contact_index];
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    if (contact.gamma(slot) != 0.0f) {
      atomicAdd(counts.data() + contact.state_offset(slot) / 3, 1);
    }
  }
}

__global__ void fill_vertex_incidence(ctd::span<const FrictionContact> contacts,
                                      ctd::span<const int> vertex_offsets,
                                      ctd::span<int> cursors,
                                      ctd::span<int> incident_contacts) {
  int contact_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (contact_index >= contacts.size()) {
    return;
  }
  const FrictionContact& contact = contacts[contact_index];
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    if (contact.gamma(slot) != 0.0f) {
      int vertex = contact.state_offset(slot) / 3;
      int local_offset = atomicAdd(cursors.data() + vertex, 1);
      incident_contacts[vertex_offsets[vertex] + local_offset] = contact_index;
    }
  }
}

__global__ void activate_contact_weights(ctd::span<const int> incidence_count,
                                         ctd::span<float> weight) {
  int state_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (state_index >= weight.size()) {
    return;
  }
  if (incidence_count[state_index / 3] == 0) {
    weight[state_index] = 0.0f;
  }
}

__global__ void summarize_incidence(ctd::span<const int> incidence_count,
                                    int* active_vertex_num,
                                    int* max_incidence) {
  int vertex = blockIdx.x * blockDim.x + threadIdx.x;
  int count = vertex < incidence_count.size() ? incidence_count[vertex] : 0;

  using BlockReduce = cub::BlockReduce<int, 128>;
  __shared__ BlockReduce::TempStorage temporary;
  int active_num = BlockReduce(temporary).Sum(count > 0 ? 1 : 0);
  if (threadIdx.x == 0) {
    atomicAdd(active_vertex_num, active_num);
  }
  __syncthreads();
  int block_max = BlockReduce(temporary).Reduce(count, cu::maximum<int>{});
  if (threadIdx.x == 0) {
    atomicMax(max_incidence, block_max);
  }
}

__global__ void initialize_aux(ctd::span<const float> state,
                               ctd::span<const float> weight,
                               ctd::span<const float> eta,
                               ctd::span<float> aux_position) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < state.size()) {
    // Unscaled ADMM contact-projection center: q ← x − W⁻¹η, then p ← q.
    float w = weight[index];
    aux_position[index] =
        w > 0.0f ? state[index] - eta[index] / w : state[index];
  }
}

__global__ void apply_correction_group(
    ctd::span<const FrictionContact> contacts,
    ctd::span<const int> grouped_contact_indices,
    ctd::span<const Vec3f> contact_correction, ctd::span<float> aux_position) {
  int local_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (local_index >= grouped_contact_indices.size()) {
    return;
  }
  int contact_index = grouped_contact_indices[local_index];
  const FrictionContact& contact = contacts[contact_index];

  // Algorithm 3 warm start restores the correction saved by contact c:
  // pⱼ ← pⱼ + γ_c,j r̄_c.
  Vec3f correction = contact_correction[contact_index];
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    if (contact.gamma(slot) == 0.0f) {
      continue;
    }
    int offset = contact.state_offset(slot);
    Vec3f position = Vec3f::vec_like(aux_position.data() + offset);
    position = vadd(position, ax(contact.gamma(slot), correction));
#pragma unroll
    for (int axis = 0; axis < 3; ++axis) {
      aux_position[offset + axis] = position(axis);
    }
  }
}

__global__ void solve_contact_group(
    ctd::span<const FrictionContact> contacts,
    ctd::span<const int> grouped_contact_indices,
    ctd::span<Vec3f> contact_correction, ctd::span<float> aux_position) {
  int local_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (local_index >= grouped_contact_indices.size()) {
    return;
  }
  int contact_index = grouped_contact_indices[local_index];
  const FrictionContact& contact = contacts[contact_index];

  // Gather the current relative contact displacement:
  // u ← Σⱼ b_c,j pⱼ + k_c.
  Vec3f u = contact.affine_offset;
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    int offset = contact.state_offset(slot);
    if (offset >= 0) {
      Vec3f position = Vec3f::vec_like(aux_position.data() + offset);
      u = vadd(u, ax(contact.coefficient(slot), position));
    }
  }

  // Remove this contact's previous contribution to obtain the locally free
  // displacement from Algorithm 3: u* ← u − r̄_c.
  Vec3f old_correction = contact_correction[contact_index];
  Vec3f u_star = vsub(u, old_correction);

  // Solve the isotropic one-contact problem so that the new u satisfies the
  // Signorini–Coulomb law.
  Vec3f u_new = solve_coulomb_contact(u_star, contact.normal, contact.friction);

  // Update the saved contact-space correction:
  // Δu ← u_new − u,  r̄_c ← r̄_c + Δu.
  Vec3f delta_u = vsub(u_new, u);
  contact_correction[contact_index] = vadd(old_correction, delta_u);

  // Map the same correction back to every movable vertex:
  // pⱼ ← pⱼ + γ_c,j Δu.
#pragma unroll
  for (int slot = 0; slot < 4; ++slot) {
    if (contact.gamma(slot) == 0.0f) {
      continue;
    }
    int offset = contact.state_offset(slot);
    Vec3f position = Vec3f::vec_like(aux_position.data() + offset);
    position = vadd(position, ax(contact.gamma(slot), delta_u));
#pragma unroll
    for (int axis = 0; axis < 3; ++axis) {
      aux_position[offset + axis] = position(axis);
    }
  }
}

__global__ void eval_contact_penalty(ctd::span<const float> weight,
                                     ctd::span<const float> aux_position,
                                     ctd::span<const float> eta,
                                     ctd::span<float> lhs_diag,
                                     ctd::span<float> rhs) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= weight.size()) {
    return;
  }
  float w = weight[index];
  if (w > 0.0f) {
    // Unscaled ADMM x-update: (H + W)x = b + Wp + η.
    lhs_diag[index] += w;
    rhs[index] += w * aux_position[index] + eta[index];
  }
}

__global__ void update_contact_multiplier(
    ctd::span<const float> state, ctd::span<const float> weight,
    ctd::span<const float> aux_position,
    ctd::span<const float> prev_aux_position, ctd::span<float> eta,
    ADMMResidualView residual) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  float local_primal_norm2 = 0.0f;
  float local_primal_scale_x2 = 0.0f;
  float local_primal_scale_aux2 = 0.0f;
  if (index < state.size() && weight[index] > 0.0f) {
    // Position formulation: state is xˡ and aux_position is projected pˡ.
    float x = state[index];
    float p = aux_position[index];
    float p_prev = prev_aux_position[index];
    float w = weight[index];

    // Unscaled ADMM dual update: ηˡ ← ηˡ⁻¹ + W(pˡ − xˡ).
    float primal_residual = x - p;
    eta[index] += w * (p - x);

    // Accumulate the primal residual xˡ − pˡ and the dual residual
    // W(pˡ − pˡ⁻¹) used by the stopping criterion.
    local_primal_norm2 = primal_residual * primal_residual;
    residual.dual_residual[index] += w * (p - p_prev);

    // The stored multiplier is already the unscaled dual ηˡ.
    float dual_scale = eta[index];
    residual.dual_scale_curr[index] += dual_scale;
    residual.dual_scale_prev[index] += dual_scale;
  }

  using BlockReduce = cub::BlockReduce<float, 128>;
  __shared__ BlockReduce::TempStorage tmp;

  // Reduce the three primal stopping-criterion terms over the block.
  float reduced = BlockReduce(tmp).Sum(local_primal_norm2);
  if (threadIdx.x == 0) {
    atomicAdd(residual.primal_norm2, reduced);
  }
  __syncthreads();
  reduced = BlockReduce(tmp).Sum(local_primal_scale_x2);
  if (threadIdx.x == 0) {
    atomicAdd(residual.primal_scale_x2, reduced);
  }
  __syncthreads();
  reduced = BlockReduce(tmp).Sum(local_primal_scale_aux2);
  if (threadIdx.x == 0) {
    atomicAdd(residual.primal_scale_aux2, reduced);
  }
}

__global__ void write_projected_state(ctd::span<const float> weight,
                                      ctd::span<const float> aux_position,
                                      ctd::span<float> state) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < state.size() && weight[index] > 0.0f) {
    state[index] = aux_position[index];
  }
}

}  // namespace

size_t ContactKeyHash::operator()(const ContactKey& key) const {
  size_t hash = static_cast<size_t>(key.type);
  auto combine = [&hash](size_t value) {
    hash ^= value + 0x9e3779b9u + (hash << 6) + (hash >> 2);
  };
  combine(key.object_id_a);
  combine(key.object_id_b);
  for (int index : key.index) {
    combine(static_cast<uint32_t>(index));
  }
  return hash;
}

__both__ Vec3f solve_coulomb_contact(const Vec3f& u_star, const Vec3f& normal,
                                     float mu) {
  float u_star_n = dot(u_star, normal);

  // Take-off: u*_N ≥ 0, so the zero-force solution is already feasible.
  if (u_star_n >= 0.0f) {
    return u_star;
  }

  Vec3f u_star_t = vsub(u_star, ax(u_star_n, normal));
  float u_star_t_norm = norm(u_star_t);
  float friction_radius = -mu * u_star_n;

  // Sticking: u* lies inside −K_μ, so the solved displacement is zero.
  if (u_star_t_norm <= friction_radius) {
    return Vec3f::zeros();
  }

  // Sliding: cancel normal motion and shrink the tangent to the cone boundary.
  return ax(1.0f - friction_radius / u_star_t_norm, u_star_t);
}

ContactConstraints::ContactConstraints(ObjRegistry& registry, int state_num,
                                       ctd::span<const Collision> collisions,
                                       CudaRuntime rt)
    : contacts_(alloc<FrictionContact>(rt, 0)),
      color_contact_indices_(alloc<int>(rt, 0)),
      base_weight_(alloc<float>(rt, state_num, 0.0f)),
      weight_(alloc<float>(rt, state_num, 0.0f)),
      aux_position_(alloc<float>(rt, state_num, 0.0f)),
      prev_aux_position_(alloc<float>(rt, state_num, 0.0f)),
      eta_(alloc<float>(rt, state_num, 0.0f)),
      contact_correction_(alloc<Vec3f>(rt, 0)) {
  Timer timer_weight("contact weight assembly", rt);
  // clang-format off
  for (uint32_t e : registry.get_entity_with_components<PhysicalState, ClothAssemblyL1Cache>()) {
    // clang-format on
    auto state = registry.get<PhysicalState>(e);
    auto cache = registry.get<ClothAssemblyL1Cache>(e);
    assert(state && cache);
    ctd::span<float> destination(base_weight_.data() + state->state_offset,
                                 state->state_num);
    cu::copy_bytes(rt.stream, *cache->contact_weight, destination);
  }
  timer_weight.end();

  append_contacts(collisions, rt);
}

int ContactConstraints::append_contacts(ctd::span<const Collision> collisions,
                                        CudaRuntime rt) {
  if (collisions.empty()) {
    return 0;
  }

  // Dedup collisions. Find true new candidates.
  std::vector<Collision> candidates = vec_like_to_host(collisions, rt);
  std::vector<Collision> new_collisions;
  for (const Collision& collision : candidates) {
    if (contact_keys_.insert(make_contact_key(collision)).second) {
      new_collisions.push_back(collision);
    }
  }
  if (new_collisions.empty()) {
    return 0;
  }

  int old_contact_num = contacts_.size();
  int new_contact_num = new_collisions.size();
  resize_buffer(old_contact_num + new_contact_num, contacts_, rt);
  resize_buffer(old_contact_num + new_contact_num, contact_correction_, rt);
  cu::fill_bytes(rt.stream,
                 ctd::span<Vec3f>(contact_correction_.data() + old_contact_num,
                                  new_contact_num),
                 0);

  auto device_collisions = vec_like_to_device<Collision>(new_collisions, rt);
  ctd::span<FrictionContact> new_contacts(contacts_.data() + old_contact_num,
                                          new_contact_num);
  int contact_grid_num = div_round_up(new_contact_num, 128);
  build_contacts<<<contact_grid_num, 128, 0, rt.stream.get()>>>(
      device_collisions, base_weight_, new_contacts);

  rebuild_partition(rt);
  return new_contact_num;
}

void ContactConstraints::rebuild_partition(CudaRuntime rt) {
  int contact_num = contacts_.size();
  assert(contact_num > 0);

  // Build CSR stlye vertex id -> contact id map.
  Timer timer_incidence("contact incidence", rt);
  int state_num = weight_.size();
  int vertex_num = state_num / 3;
  auto incidence_count = alloc<int>(rt, vertex_num + 1, 0);
  int contact_grid_num = div_round_up(contact_num, 128);
  count_vertex_incidence<<<contact_grid_num, 128, 0, rt.stream.get()>>>(
      contacts_, incidence_count);
  auto vertex_offsets = alloc<int>(rt, vertex_num + 1);
  size_t scan_temporary_size = 0;
  cub::DeviceScan::ExclusiveSum(nullptr, scan_temporary_size,
                                incidence_count.data(), vertex_offsets.data(),
                                vertex_num + 1, rt.stream.get());
  auto scan_temporary = alloc<char>(rt, scan_temporary_size);
  cub::DeviceScan::ExclusiveSum(scan_temporary.data(), scan_temporary_size,
                                incidence_count.data(), vertex_offsets.data(),
                                vertex_num + 1, rt.stream.get());
  int incidence_num = scalar_load(vertex_offsets.data() + vertex_num, rt);
  auto incident_contacts = alloc<int>(rt, incidence_num);
  auto cursors = alloc<int>(rt, vertex_num, 0);
  fill_vertex_incidence<<<contact_grid_num, 128, 0, rt.stream.get()>>>(
      contacts_, vertex_offsets, cursors, incident_contacts);

  auto active_vertex_num = alloc<int>(rt, 1, 0);
  auto max_incidence_device = alloc<int>(rt, 1, 0);
  int vertex_grid_num = div_round_up(vertex_num, 128);
  summarize_incidence<<<vertex_grid_num, 128, 0, rt.stream.get()>>>(
      ctd::span<const int>(incidence_count.data(), vertex_num),
      active_vertex_num.data(), max_incidence_device.data());
  active_dof_ = 3 * scalar_load(active_vertex_num.data(), rt);
  int max_incidence = scalar_load(max_incidence_device.data(), rt);
  cu::copy_bytes(rt.stream, base_weight_, weight_);
  int state_grid_num = div_round_up(state_num, 128);
  activate_contact_weights<<<state_grid_num, 128, 0, rt.stream.get()>>>(
      incidence_count, weight_);
  timer_incidence.end();

  Timer timer_coloring("contact coloring", rt);
  auto [grouped_indices, color_offsets] =
      color_contacts(contacts_, vertex_offsets, incident_contacts, rt);
  color_contact_indices_ = std::move(grouped_indices);
  color_contact_offsets_ = std::move(color_offsets);
  timer_coloring.end();
  int max_group_size = 0;
  for (int color = 0; color < get_color_num(); ++color) {
    max_group_size =
        std::max(max_group_size, color_contact_offsets_[color + 1] -
                                     color_contact_offsets_[color]);
  }
  SPDLOG_DEBUG(
      "Daviet contacts {}, active dof {}, max incidence {}, colors {}, max "
      "group {}",
      contacts_.size(), active_dof_, max_incidence, get_color_num(),
      max_group_size);
}

void ContactConstraints::eval(ctd::span<float> lhs_diag, ctd::span<float> rhs,
                              CudaRuntime rt) const {
  int grid_num = div_round_up(weight_.size(), 128);
  eval_contact_penalty<<<grid_num, 128, 0, rt.stream.get()>>>(
      weight_, aux_position_, eta_, lhs_diag, rhs);
}

void ContactConstraints::apply_contact_corrections(CudaRuntime rt) {
  // Algorithm 3 warm start. Colors are sequential Gauss–Seidel stages;
  // contacts inside one color update disjoint vertices in parallel.
  for (int color = 0; color < get_color_num(); ++color) {
    int begin = color_contact_offsets_[color];
    int end = color_contact_offsets_[color + 1];
    ctd::span<const int> group(color_contact_indices_.data() + begin,
                               end - begin);
    int grid_num = div_round_up(group.size(), 128);
    apply_correction_group<<<grid_num, 128, 0, rt.stream.get()>>>(
        contacts_, group, contact_correction_, aux_position_);
  }
}

void ContactConstraints::solve_contacts(CudaRuntime rt) {
  // Algorithm 3 matrix-free Gauss–Seidel projection. Each contact consumes
  // the position updates produced by all preceding colors and sweeps.
  for (int sweep = 0; sweep < sweep_num_; ++sweep) {
    for (int color = 0; color < get_color_num(); ++color) {
      int begin = color_contact_offsets_[color];
      int end = color_contact_offsets_[color + 1];
      ctd::span<const int> group(color_contact_indices_.data() + begin,
                                 end - begin);
      int grid_num = div_round_up(group.size(), 128);
      solve_contact_group<<<grid_num, 128, 0, rt.stream.get()>>>(
          contacts_, group, contact_correction_, aux_position_);
    }
  }
}

void ContactConstraints::update_aux_var_and_lagrange_mul(
    ctd::span<const float> state, ADMMResidualView residual, CudaRuntime rt) {
  // Preserve pˡ⁻¹ for the dual residual W(pˡ − pˡ⁻¹).
  cu::copy_bytes(rt.stream, aux_position_, prev_aux_position_);
  int grid_num = div_round_up(state.size(), 128);

  // Start the contact projection from q ← x − W⁻¹η, then p ← q.
  initialize_aux<<<grid_num, 128, 0, rt.stream.get()>>>(state, weight_, eta_,
                                                        aux_position_);

  // Warm start Algorithm 3 with r̄ from the preceding ADMM iteration, then
  // perform the colored Gauss–Seidel projection.
  apply_contact_corrections(rt);
  solve_contacts(rt);

  // Finish ADMM Step (iii) and accumulate convergence residuals.
  update_contact_multiplier<<<grid_num, 128, 0, rt.stream.get()>>>(
      state, weight_, aux_position_, prev_aux_position_, eta_, residual);
}

void ContactConstraints::project(ctd::span<float> state, CudaRuntime rt) {
  int grid_num = div_round_up(state.size(), 128);
  write_projected_state<<<grid_num, 128, 0, rt.stream.get()>>>(
      weight_, aux_position_, state);
}

}  // namespace silk::cuda
