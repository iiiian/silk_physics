#include "backend/cuda/object_state.cuh"

#include <cuda_runtime_api.h>

#include <cassert>
#include <utility>
#include <vector>

#include "backend/cuda/collision/bbox.cuh"
#include "backend/cuda/collision/broadphase.cuh"
#include "backend/cuda/copy_vector_like.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "common/mesh.hpp"

namespace silk::cuda {

namespace {

struct VertexCollider {
  Bbox bbox;
  int vertex = 0;
};

}  // namespace

ObjectState::ObjectState(int state_offset, ctd::span<const float> curr_state,
                         ctd::span<const float> state_velocity, CudaRuntime rt)
    : state_offset(state_offset),
      state_num(static_cast<int>(curr_state.size())),
      curr_state(rt.stream, rt.mem_resource, curr_state),
      state_velocity(rt.stream, rt.mem_resource, state_velocity),
      perm(rt.stream, rt.mem_resource),
      inv_perm(rt.stream, rt.mem_resource) {
  assert(curr_state.size() == state_velocity.size());
  assert(state_offset >= 0);
}

ObjectState::ObjectState(int state_offset, const ::silk::TriMesh& mesh) {
  assert(state_offset >= 0);

  // TODO: impl after broadphase fin

  const auto& V = mesh.V;
  int vert_num = static_cast<int>(V.rows());
  int state_num_local = 3 * vert_num;

  // Build KDTree over rest positions to compute permutation.
  std::vector<VertexCollider> colliders(static_cast<size_t>(vert_num));
  float bbox_padding = 0.05f * mesh.avg_edge_length;

  Eigen::Vector3f min = V.colwise().minCoeff();
  Eigen::Vector3f max = V.colwise().maxCoeff();
  Bbox root_bbox{min, max};
  root_bbox.pad_inplace(bbox_padding);

  for (int i = 0; i < vert_num; ++i) {
    VertexCollider& vc = colliders[static_cast<size_t>(i)];
    vc.vertex = i;
    Eigen::Vector3f p = V.row(i);
    vc.bbox.min = p;
    vc.bbox.max = p;
    vc.bbox.pad_inplace(bbox_padding);
  }

  using VertexTree = KDTree<VertexCollider, 4>;
  VertexTree tree;
  tree.init(std::move(colliders));
  tree.update(root_bbox);

  const std::vector<int>& proxies = tree.get_proxies();
  const std::vector<VertexCollider>& tree_colliders = tree.get_colliders();

  perm.resize(vert_num);
  inv_perm.resize(vert_num);
  for (int new_idx = 0; new_idx < vert_num; ++new_idx) {
    int collider_idx = proxies[static_cast<size_t>(new_idx)];
    int old_vertex = tree_colliders[static_cast<size_t>(collider_idx)].vertex;
    perm(new_idx) = old_vertex;
    inv_perm(old_vertex) = new_idx;
  }

  // Initialize device state in permuted order.
  Eigen::VectorXf curr_state(state_num_local);
  Eigen::VectorXf flat_V = V.reshaped<Eigen::RowMajor>();
  for (int new_idx = 0; new_idx < vert_num; ++new_idx) {
    int old_idx = perm(new_idx);
    auto dst_seq = Eigen::seqN(3 * new_idx, 3);
    auto src_seq = Eigen::seqN(3 * old_idx, 3);
    curr_state(dst_seq) = flat_V(src_seq);
  }
  Eigen::VectorXf state_velocity = Eigen::VectorXf::Zero(state_num_local);

  this->state_num = state_num_local;
  this->state_offset = state_offset;
  d_curr_state = host_eigen_to_device(curr_state);
  d_state_velocity = host_eigen_to_device(state_velocity);
}

}  // namespace silk::cuda
