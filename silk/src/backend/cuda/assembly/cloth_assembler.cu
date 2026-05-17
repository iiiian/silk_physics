#include "backend/cuda/assembly/cloth_assembler.cuh"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <cuda/algorithm>
#include <cuda/std/span>
#include <memory>
#include <vector>

#include "backend/cuda/assembly/cloth_assembly_l1_cache.cuh"
#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "backend/cuda/physical_state.cuh"
#include "backend/cuda/pin.hpp"
#include "backend/cuda/solver/cloth_admm_helper.cuh"
#include "common/cloth_assembly_l2_cache.hpp"
#include "common/initial_state.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

namespace {

TriMesh build_permuted_mesh(const TriMesh& mesh, ctd::span<int> perm,
                            ctd::span<int> inv_perm) {
  TriMesh perm_mesh = mesh;
  int vert_num = mesh.V.rows();
  for (int new_idx = 0; new_idx < vert_num; ++new_idx) {
    int old_idx = inv_perm[new_idx];
    perm_mesh.V.row(new_idx) = mesh.V.row(old_idx);
  }
  for (int i = 0; i < mesh.E.rows(); ++i) {
    perm_mesh.E(i, 0) = inv_perm[mesh.E(i, 0)];
    perm_mesh.E(i, 1) = inv_perm[mesh.E(i, 1)];
  }
  for (int i = 0; i < mesh.F.rows(); ++i) {
    perm_mesh.F(i, 0) = inv_perm[mesh.F(i, 0)];
    perm_mesh.F(i, 1) = inv_perm[mesh.F(i, 1)];
    perm_mesh.F(i, 2) = inv_perm[mesh.F(i, 2)];
  }
  return perm_mesh;
}

PinIndex build_permuted_pin_index(const PinIndex& pin, ctd::span<int> perm) {
  PinIndex perm_pin = pin;

  if (!pin.is_all_pinned) {
    for (int& index : perm_pin.index) {
      index = perm[index];
    }
  }

  return perm_pin;
}

}  // namespace

void assemble_cloth(ObjRegistry& registry, uint32_t entity, float dt,
                    int state_offset, CudaRuntime rt) {
  auto& e = entity;

  auto cloth_config = registry.get<ClothConfig>(e);
  auto collision_config = registry.get<CollisionConfigPlus>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin_index = registry.get<PinIndex>(e);
  auto init_state = registry.get<InitialState>(e);

  // Cloth entity sanity check.
  assert(cloth_config && collision_config && mesh && pin_index && init_state);

  // Prepare pin position.
  auto pin_pos = registry.get<PinPosition>(e);
  if (!pin_pos) {
    pin_pos = registry.set(e, PinPosition{*pin_index, init_state->position});
  }
  assert(pin_pos != nullptr);

  // Prepare MeshPartition.
  auto part = registry.get<MeshPartition>(e);
  if (!part) {
    part = registry.set(e, MeshPartition{*mesh, rt});
  }
  assert(part != nullptr);

  // Prepare PhysicalState.
  auto state = registry.get<PhysicalState>(e);
  if (!state) {
    auto init_pos = host_eigen_to_device(init_state->position, rt);
    auto perm_pos = alloc<float>(rt, init_state->position.size());
    part->permute(init_pos, perm_pos, rt);
    auto init_vel = host_eigen_to_device(init_state->velocity, rt);
    auto perm_vel = alloc<float>(rt, init_state->velocity.size());
    part->permute(init_vel, perm_vel, rt);

    PhysicalState tmp;
    tmp.state_offset = state_offset;
    tmp.state_num = perm_pos.size();
    tmp.curr_state = std::move(perm_pos);
    tmp.state_velocity = std::move(perm_vel);

    state = registry.set(e, std::move(tmp));
  } else {
    state->state_offset = state_offset;
  }
  assert(state != nullptr);

  std::unique_ptr<TriMesh> perm_mesh;
  std::unique_ptr<PinIndex> perm_pin;

  // Prepare L2 assembly cache.
  auto l2_cache = registry.get<ClothAssemblyL2Cache>(e);
  if (!l2_cache) {
    if (!perm_mesh) {
      perm_mesh = std::make_unique<TriMesh>(
          build_permuted_mesh(*mesh, part->h_perm, part->h_inv_perm));
    }
    l2_cache = registry.set(e, ClothAssemblyL2Cache{*cloth_config, *perm_mesh});
  }
  assert(l2_cache != nullptr);

  // Prepare L1 assembly cache.
  auto l1_cache = registry.get<ClothAssemblyL1Cache>(e);
  if (!(l1_cache && l1_cache->dt == dt)) {
    l1_cache = registry.set(
        e, ClothAssemblyL1Cache{*cloth_config, *part, *l2_cache, dt, rt});
  }
  assert(l1_cache != nullptr);

  auto admm_helper = registry.get<ClothADMMHelper>(e);
  if (!admm_helper) {
    admm_helper = registry.set(
        e, ClothADMMHelper{l1_cache->vert_num, l1_cache->face_num, rt});
  } else {
    admm_helper->reset_aux_lagrange_mul(rt);
  }

  // Prepare ObjCollider.
  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    if (!perm_mesh) {
      perm_mesh = std::make_unique<TriMesh>(
          build_permuted_mesh(*mesh, part->h_perm, part->h_inv_perm));
    }
    if (!perm_pin) {
      perm_pin = std::make_unique<PinIndex>(
          build_permuted_pin_index(*pin_index, part->h_perm));
    }

    // Per-vertex mass in permuted indexing for collision.
    Eigen::VectorXf collider_mass = cloth_config->density * l2_cache->mass;

    std::vector<float> perm_init_pos(init_state->position.size());
    ctd::span<const float> init_pos_span{init_state->position.data(),
                                         init_state->position.size()};
    part->permute(init_pos_span, perm_init_pos, rt);

    auto new_collider = ObjectCollider::from_physical(
        *collision_config, *perm_mesh, *perm_pin, perm_init_pos, collider_mass,
        state_offset, rt);
    collider = registry.set(e, std::move(new_collider));
  } else {
    collider->update_state_offset(state_offset, rt);
  }
  assert(collider != nullptr);
}

}  // namespace silk::cuda
