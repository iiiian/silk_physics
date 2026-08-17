#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cuda/buffer>
#include <cuda/std/span>
#include <unordered_set>
#include <vector>

#include "backend/cuda/collision/collision.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "backend/cuda/ecs.hpp"
#include "backend/cuda/simple_linalg.cuh"

namespace silk::cuda {

struct FrictionContact {
  /// Global offset of each contact vertex's xyz block. A value of
  /// −1 denotes a kinematic obstacle vertex..
  Vec4i state_offset;

  /// Contact Jacobian coefficients b_c,j from the paper.
  Vec4f coefficient;

  /// Precomputed Algorithm 3 update factors
  /// γ_c,j = b_c,j / (s_c w_j), where s_c = Σ_j b_c,j² / w_j. A zero entry
  /// excludes a pinned or kinematic vertex from position updates.
  Vec4f gamma;

  /// Contact normal n_c, directed from contact body B toward body A.
  Vec3f normal;

  /// Affine contact term k_c. It contains kinematic vertex positions and the
  /// offset −d̂ n_c, so Σ_j b_c,j p_j + k_c measures displacement relative to
  /// the activation distance d̂.
  Vec3f affine_offset;

  /// Coulomb friction coefficient μ_c.
  float friction;
};

/// Exact analytical Signorini--Coulomb solve for one contact.
__both__ Vec3f solve_coulomb_contact(const Vec3f& u_star, const Vec3f& normal,
                                     float mu);

struct ContactKey {
  CollisionType type;
  uint32_t object_id_a;
  uint32_t object_id_b;
  std::array<int, 4> index;

  bool operator==(const ContactKey&) const = default;
};

struct ContactKeyHash {
  size_t operator()(const ContactKey& key) const;
};

class ContactConstraints {
 public:
  ContactConstraints(ObjRegistry& registry, int state_num,
                     ctd::span<const Collision> collisions, CudaRuntime rt);

  int get_active_dof() const { return active_dof_; }
  int get_contact_num() const { return contacts_.size(); }
  int get_color_num() const { return color_contact_offsets_.size() - 1; }
  /// Append contacts to active set.
  int append_contacts(ctd::span<const Collision> collisions, CudaRuntime rt);

  /// Eval constraint for ADMM main.
  void eval(ctd::span<float> lhs_diag, ctd::span<float> rhs,
            CudaRuntime rt) const;

  void update_aux_var_and_lagrange_mul(
      ctd::span<const float> state, ctd::span<float> primal_norm2,
      ctd::span<float> primal_scale_x2, ctd::span<float> primal_scale_aux2,
      ctd::span<float> dual_residual, ctd::span<float> dual_scale_curr,
      ctd::span<float> dual_scale_prev, CudaRuntime rt);

  /// Project vertices to feasible states.
  void project(ctd::span<float> state, CudaRuntime rt);

 private:
  void rebuild_partition(CudaRuntime rt);
  void apply_contact_corrections(CudaRuntime rt);
  void solve_contacts(CudaRuntime rt);

  int active_dof_ = 0;
  int sweep_num_ = 10;  //< Gauss–Seidel sweeps.
  cu::device_buffer<FrictionContact> contacts_;

  /// Contact indices grouped by color in CSR-style storage.
  cu::device_buffer<int> color_contact_indices_;
  /// CSR-style offsets delimiting each color group.
  std::vector<int> color_contact_offsets_{0};
  /// Per-DoF contact weights before inactive DoFs are removed.
  cu::device_buffer<float> base_weight_;
  /// Active per-DoF contact weights; zero for DoFs without contacts.
  cu::device_buffer<float> weight_;
  /// Feasible auxiliary position p produced by contact projection.
  cu::device_buffer<float> aux_position_;
  cu::device_buffer<float> prev_aux_position_;
  /// Unscaled ADMM multiplier η for the equality p − x = 0.
  cu::device_buffer<float> eta_;
  /// Per-contact correction memory r̄ used by Algorithm 3.
  cu::device_buffer<Vec3f> contact_correction_;
  std::unordered_set<ContactKey, ContactKeyHash> contact_keys_;
};

}  // namespace silk::cuda
