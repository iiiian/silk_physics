#pragma once

#include <Eigen/Core>
#include <optional>

#include "backend/cuda/csr_matrix.hpp"
#include "backend/cuda/object_state.hpp"
#include "common/cloth_topology.hpp"
#include "common/mesh.hpp"
#include "common/pin.hpp"
#include "silk/silk.hpp"

namespace silk::cuda {

/**
 * Dynamic, time step or config dependent quantities used by the cloth solver.
 *
 * Notation:
 * state_num = 3 * vertex num.
 */
class ClothSolverContext {
 public:
  float dt;
  int state_num = 0;
  int face_num = 0;
  Eigen::VectorXf h_mass;
  float* d_mass = nullptr;
  float* d_area = nullptr;
  float* d_D = nullptr;
  float* d_DB = nullptr;
  CSRMatrix d_R;
  int* d_F = nullptr;
  float* d_jacobian_ops = nullptr;
  float* d_C0 = nullptr;

  int r = 0;
  Eigen::MatrixXf UHU;
  float* d_U = nullptr;
  // Row-major copy of U (state_num x r) for subspace kernels that
  // benefit from contiguous per-row access.
  float* d_U_RM = nullptr;
  float* d_HX = nullptr;
  float* d_X = nullptr;

 public:
  ClothSolverContext() = default;
  ClothSolverContext(const ClothConfig& config, const TriMesh& mesh,
                     const ClothTopology& topology, const Pin& pin,
                     const ObjectState& state, float dt);
  ClothSolverContext(const ClothSolverContext& other) = delete;
  ClothSolverContext(ClothSolverContext&& other) noexcept;
  ClothSolverContext& operator=(const ClothSolverContext& other) = delete;
  ClothSolverContext& operator=(ClothSolverContext&& other) noexcept;
  ~ClothSolverContext();

 private:
  void swap(ClothSolverContext& other) noexcept;
};

}  // namespace silk::cuda
