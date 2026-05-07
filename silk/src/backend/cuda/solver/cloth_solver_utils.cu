#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>
#include <memory>

#include "backend/cuda/collision/object_collider.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/mesh_partition.cuh"
#include "backend/cuda/physical_state.cuh"
// #include "backend/cuda/solver/a_jacobi_solver.hpp"
// #include "backend/cuda/solver/barrier_constrain.hpp"
#include "backend/cuda/eigen_cuda_interop.cuh"
#include "backend/cuda/graph_partition.hpp"
#include "backend/cuda/solver/cloth_solver_context.cuh"
#include "backend/cuda/solver/cloth_solver_kernel.cuh"
#include "backend/cuda/solver/cloth_solver_utils.cuh"
// #include "backend/cuda/solver/inexact_solver.hpp"
#include <cuda/algorithm>
#include <cuda/std/span>

#include "common/cloth_topology.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

namespace {

TriMesh build_permuted_mesh(const TriMesh& mesh, ctd::span<int> perm,
                            ctd::span<int> inv_perm) {
  TriMesh perm_mesh = mesh;
  int vert_num = mesh.V.rows();
  for (int i = 0; i < vert_num; ++i) {
    int old_idx = perm[i];
    perm_mesh.V.row(i) = mesh.V.row(old_idx);
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

Pin build_permuted_pin(const Pin& pin, ctd::span<int> perm,
                       ctd::span<int> inv_perm) {
  Pin perm_pin = pin;

  if (!pin.is_all_pinned) {
    for (int i = 0; i < pin.index.size(); ++i) {
      perm_pin.index(i) = perm[pin.index(i)];
    }
    return perm_pin;
  }

  for (int i = 0; i < pin.curr_position.size() / 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      int old_id = inv_perm[i];
      perm_pin.curr_position(3 * i + j) = pin.curr_position(3 * old_id + j);
      perm_pin.prev_position(3 * i + j) = pin.prev_position(3 * old_id + j);
    }
  }
}

}  // namespace

void batch_reset_cloth_simulation(ObjRegistry& registry, CudaRuntime rt) {
  auto entities =
      registry.get_entity_with_components<TriMesh, PhysicalState, MeshPartition,
                                          ClothSolverContext>();
  for (uint32_t e : entities) {
    auto mesh = registry.get<TriMesh>(e);
    auto part = registry.get<MeshPartition>(e);
    auto state = registry.get<PhysicalState>(e);
    auto context = registry.get<ClothSolverContext>(e);

    // Reset current position to mesh position.
    auto original_pos = host_eigen_to_device(mesh->V, rt);
    part->permute(original_pos, *state->curr_state, rt);
    // Reset current velocity to zero.
    cu::fill_bytes(rt.stream, *(state->state_velocity), 0);
  }
}

bool prepare_cloth_simulation(ObjRegistry& registry, uint32_t entity, float dt,
                              int state_offset, CudaRuntime rt) {
  auto& e = entity;

  auto cloth_config = registry.get<ClothConfig>(e);
  auto collision_config = registry.get<CollisionConfigPlus>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin = registry.get<Pin>(e);

  // Cloth entity sanity check.
  assert(cloth_config && collision_config && mesh && pin);

  auto part = registry.get<MeshPartition>(e);
  if (!part) {
    part = registry.set(e, MeshPartition{*mesh, rt});
  }
  assert(part != nullptr);

  auto state = registry.get<PhysicalState>(e);
  if (!state) {
    auto mesh_pos = host_eigen_to_device(mesh->V, rt);
    auto perm_pos = alloc<float>(rt, mesh->V.rows());
    part->permute(mesh_pos, perm_pos, rt);
    PhysicalState tmp;
    tmp.state_offset = state_offset;
    tmp.state_num = perm_pos.size();
    tmp.curr_state = std::move(perm_pos);
    tmp.state_velocity = alloc<float>(rt, tmp.state_num, 0);
    state = registry.set<PhysicalState>(e, std::move(tmp));
  } else {
    state->state_offset = state_offset;
  }
  assert(state != nullptr);

  std::unique_ptr<TriMesh> perm_mesh;
  std::unique_ptr<Pin> perm_pin;

  auto topology = registry.get<ClothTopology>(e);
  if (!topology) {
    if (!perm_mesh) {
      perm_mesh = std::make_unique<TriMesh>(
          build_permuted_mesh(*mesh, part->h_perm, part->h_inv_perm));
    }
    topology = registry.set<ClothTopology>(
        e, ClothTopology{*cloth_config, *perm_mesh});
  }
  assert(topology != nullptr);

  auto context = registry.get<ClothSolverContext>(e);
  if (!(context && context->dt == dt)) {
    context = registry.set<ClothSolverContext>(
        e, ClothSolverContext{*cloth_config, *topology, dt, rt});
  }
  assert(context != nullptr);

  using ObjectCollider = ::silk::cuda::collision::ObjectCollider;
  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    if (!perm_mesh) {
      perm_mesh = std::make_unique<TriMesh>(
          build_permuted_mesh(*mesh, part->h_perm, part->h_inv_perm));
    }
    if (!perm_pin) {
      perm_pin = std::make_unique<Pin>(
          build_permuted_pin(*pin, part->h_perm, part->h_inv_perm));
    }
    // Per-vertex mass in permuted indexing for collision.
    Eigen::VectorXf collider_mass = cloth_config->density * topology->mass;

    auto new_collider =
        ObjectCollider::from_physical(*collision_config, *perm_mesh, *perm_pin,
                                      collider_mass, state_offset, rt);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  } else {
    collider->update_state_offset(state_offset, rt);
  }
  assert(collider != nullptr);

  return true;
}

void compute_cloth_invariant_rhs(const ClothSolverContext& solver_context,
                                 const Pin& pin,
                                 const PhysicalState& physical_state,
                                 float* d_rhs) {
  auto& c = solver_context;

  // set pin rhs
  Eigen::VectorXf pin_rhs = Eigen::VectorXf::Zero(c.state_num);
  for (int i = 0; i < pin.index.size(); ++i) {
    int v_old = pin.index(i);
    int v_new = physical_state.inv_perm(v_old);
    pin_rhs(Eigen::seqN(3 * v_new, 3)) =
        pin.pin_stiffness * pin.position(Eigen::seqN(3 * i, 3));
  }
  CHECK_CUDA(cudaMemcpy(d_rhs, pin_rhs.data(), c.state_num * sizeof(float),
                        cudaMemcpyHostToDevice));

  // set rest curvature rhs
  vector_add(c.state_num, d_rhs, c.d_C0, d_rhs);
}

void batch_compute_cloth_invariant_rhs(ObjRegistry& registry, float* d_rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<ClothSolverContext>(e);
    auto pin = registry.get<Pin>(e);

    if (state && context && pin) {
      compute_cloth_invariant_rhs(*context, *pin, *state,
                                  d_rhs + state->state_offset);
    }
  }
  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());
}

bool compute_cloth_outer_loop(const float* d_state,
                              const float* d_state_velocity,
                              const float* d_barrier_lhs,
                              const float* d_barrier_rhs,
                              const Eigen::Vector3f& state_acceleration,
                              ClothSolverContext& solver_context,
                              float* d_rhs) {
  auto& s = solver_context;

  // Barrier constraint update
  vector_add(s.state_num, s.d_D, d_barrier_lhs, s.d_DB);
  compute_outer_rhs(s.state_num, s.dt, state_acceleration(0),
                    state_acceleration(1), state_acceleration(2), s.d_mass,
                    d_state, d_state_velocity, d_barrier_rhs, d_rhs);

  return true;
}

bool batch_compute_cloth_outer_loop(ObjRegistry& registry, const float* d_state,
                                    const float* d_state_velocity,
                                    const BarrierConstrain& barrier_constrain,
                                    const Eigen::Vector3f& state_acceleration,
                                    float* d_rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto obj_state = registry.get<ObjectState>(e);
    auto solver_ctx = registry.get<ClothSolverContext>(e);

    if (obj_state && solver_ctx) {
      int offset = obj_state->state_offset;
      const float* d_lhs = barrier_constrain.d_lhs + offset;
      const float* d_rhs_ptr = barrier_constrain.d_rhs + offset;
      if (!compute_cloth_outer_loop(d_state + offset, d_state_velocity + offset,
                                    d_lhs, d_rhs_ptr, state_acceleration,
                                    *solver_ctx, d_rhs + offset)) {
        return false;
      }
    }
  }
  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  return true;
}

bool compute_cloth_inner_loop(const ClothConfig& config,
                              const ClothSolverContext& solver_context,
                              const BarrierConstrain& barrier_constrain,
                              int state_offset, const float* d_outer_rhs,
                              float* d_state) {
  auto& s = solver_context;

  DynArray<float> d_inner_rhs(s.state_num);
  CHECK_CUDA(cudaMemcpy(d_inner_rhs, d_outer_rhs, s.state_num * sizeof(float),
                        cudaMemcpyDeviceToDevice));
  compute_elastic_rhs(s.face_num, config.elastic_stiffness, s.d_F, d_state,
                      s.d_jacobian_ops, s.d_area, d_inner_rhs);

  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  // inexact_solve(solver_context, d_inner_rhs, barrier_constrain, state_offset,
  //               d_state);
  // CHECK_CUDA(cudaGetLastError());

  bool success = a_jacobi(s.state_num, 20, 1e-6f, 1e-3f, s.d_R, s.d_DB,
                          d_inner_rhs, d_state);
  if (!success) {
    SPDLOG_ERROR("A-Jacobi solve failed.");
    return false;
  }

  return true;
}

bool batch_compute_cloth_inner_loop(ObjRegistry& registry,
                                    const float* d_outer_rhs,
                                    const BarrierConstrain& barrier_constrain,
                                    float* d_state) {
  for (Entity& e : registry.get_all_entities()) {
    auto config = registry.get<ClothConfig>(e);
    auto mesh = registry.get<TriMesh>(e);
    auto solver_context = registry.get<ClothSolverContext>(e);
    auto state = registry.get<ObjectState>(e);

    if (!(config && mesh && solver_context && state)) {
      continue;
    }

    int offset = state->state_offset;
    if (!compute_cloth_inner_loop(*config, *solver_context, barrier_constrain,
                                  offset, d_outer_rhs + offset,
                                  d_state + offset)) {
      return false;
    }
  }
  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  return true;
}
}  // namespace silk::cuda
