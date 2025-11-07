#pragma once

#include <cuBQL/math/box.h>

#include <Eigen/Core>
#include <vector>

namespace silk::gpu {

struct EdgeCollider {
  Eigen::Vector2i index;
  Eigen::Vector2f inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix<float, 3, 2> position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix<float, 3, 2> position_t1;
};

struct TriangleCollider {
  Eigen::Vector3i index;
  Eigen::Vector3f inv_mass;
  // position of vertex at t0, each col is a vertex
  Eigen::Matrix<float, 3, 3> position_t0;
  // position of vertex at t1, each col is a vertex
  Eigen::Matrix<float, 3, 3> position_t1;
};

class BVHContext {
 public:
  int edge_num = 0;
  cuBQL::box3f* d_edge_bboxes = nullptr;
  EdgeCollider* d_edge_colliders = nullptr;

  int triangle_num = 0;
  cuBQL::box3f* d_triangle_bboxes = nullptr;
  TriangleCollider* d_triangle_colliders = nullptr;

 public:
  static BVHContext make_bvh_context(
      const std::vector<cuBQL::box3f>& edge_bboxes,
      const std::vector<EdgeCollider>& edge_colliders,
      const std::vector<cuBQL::box3f>& triangle_bboxes,
      const std::vector<TriangleCollider>& triangle_colliders);

  BVHContext() = default;
  BVHContext(const BVHContext& other) = delete;
  BVHContext(BVHContext&& other) noexcept = default;
  BVHContext& operator=(const BVHContext& other) = delete;
  BVHContext& operator=(BVHContext&& other) = default;
  ~BVHContext();
};

}  // namespace silk::gpu
