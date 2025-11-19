#include "backend/cuda/solver/cloth_solver_utils.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>

#include "backend/cuda/collision/object_collider.hpp"
#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/device_vector.hpp"
#include "backend/cuda/object_state.hpp"
#include "backend/cuda/solver/a_jacobi_solver.hpp"
#include "backend/cuda/solver/barrier_constrain.hpp"
#include "backend/cuda/solver/cloth_solver_context.hpp"
#include "backend/cuda/solver/cloth_solver_kernel.hpp"
#include "backend/cuda/solver/inexact_solver.hpp"
#include "common/cloth_topology.hpp"
#include "common/logger.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

void batch_reset_cloth_simulation(Registry& registry) {
  for (Entity& e : registry.get_all_entities()) {
    auto mesh = registry.get<TriMesh>(e);
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<ClothSolverContext>(e);

    if (mesh && state && context) {
      auto vec_V = mesh->V.reshaped<Eigen::RowMajor>();
      *state = ObjectState{0, vec_V, Eigen::VectorXf::Zero(vec_V.size())};
    }
  }
}

bool prepare_cloth_simulation(Registry& registry, Entity& entity, float dt,
                              int state_offset) {
  auto& e = entity;

  auto cloth_config = registry.get<ClothConfig>(e);
  auto collision_config = registry.get<CollisionConfig>(e);
  auto mesh = registry.get<TriMesh>(e);
  auto pin = registry.get<Pin>(e);

  // Cloth entity sanity check.
  assert(cloth_config && collision_config && mesh && pin);

  auto state = registry.get<ObjectState>(e);
  if (!state) {
    auto vec_V = mesh->V.reshaped<Eigen::RowMajor>();
    state = registry.set<ObjectState>(
        e,
        ObjectState{state_offset, vec_V, Eigen::VectorXf::Zero(vec_V.size())});
  } else {
    state->state_offset = state_offset;
  }
  assert(state != nullptr);

  auto topology = registry.get<ClothTopology>(e);
  if (!topology) {
    topology =
        registry.set<ClothTopology>(e, ClothTopology(*cloth_config, *mesh));
  }
  assert(topology != nullptr);

  auto context = registry.get<ClothSolverContext>(e);
  if (!(context && context->dt == dt)) {
    context = registry.set<ClothSolverContext>(
        e, ClothSolverContext{*cloth_config, *mesh, *topology, *pin, dt});
  }
  assert(context != nullptr);

  auto collider = registry.get<ObjectCollider>(e);
  if (!collider) {
    auto new_collider = ObjectCollider(e.self, *collision_config, *mesh, *pin,
                                       context->h_mass, state_offset);
    collider = registry.set<ObjectCollider>(e, std::move(new_collider));
  } else {
    collider->state_offset = state_offset;
  }
  assert(collider != nullptr);

  return true;
}

void compute_cloth_invariant_rhs(const ClothSolverContext& solver_context,
                                 const Pin& pin, float* d_rhs) {
  auto& c = solver_context;

  // set pin rhs
  Eigen::VectorXf pin_rhs = Eigen::VectorXf::Zero(c.state_num);
  for (int i = 0; i < pin.index.size(); ++i) {
    pin_rhs(Eigen::seqN(3 * pin.index(i), 3)) =
        pin.pin_stiffness * pin.position(Eigen::seqN(3 * i, 3));
  }
  CHECK_CUDA(cudaMemcpy(d_rhs, pin_rhs.data(), c.state_num * sizeof(float),
                        cudaMemcpyHostToDevice));

  // set rest curvature rhs
  vector_add(c.state_num, d_rhs, c.d_C0, d_rhs);
}

void batch_compute_cloth_invariant_rhs(Registry& registry, float* d_rhs) {
  for (Entity& e : registry.get_all_entities()) {
    auto state = registry.get<ObjectState>(e);
    auto context = registry.get<ClothSolverContext>(e);
    auto pin = registry.get<Pin>(e);

    if (state && context && pin) {
      compute_cloth_invariant_rhs(*context, *pin, d_rhs + state->state_offset);
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

bool batch_compute_cloth_outer_loop(Registry& registry, const float* d_state,
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

  DVector<float> d_inner_rhs(s.state_num);
  CHECK_CUDA(cudaMemcpy(d_inner_rhs, d_outer_rhs, s.state_num * sizeof(float),
                        cudaMemcpyDeviceToDevice));
  compute_elastic_rhs(s.face_num, config.elastic_stiffness, s.d_F, d_state,
                      s.d_jacobian_ops, s.d_area, d_inner_rhs);

  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  // inexact_solve(solver_context, d_inner_rhs, barrier_constrain, state_offset,
  //               d_state);
  // CHECK_CUDA(cudaGetLastError());

  bool success = a_jacobi(s.state_num, 2000, 1e-6f, 1e-3f, s.d_R, s.d_DB,
                          d_inner_rhs, d_state);
  if (!success) {
    SPDLOG_ERROR("A-Jacobi solve failed.");
    return false;
  }

  return true;
}

bool batch_compute_cloth_inner_loop(Registry& registry,
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
