#pragma once

#include <Eigen/Core>
#include <optional>

#include "collision.hpp"

namespace silk {

struct CCDConfig {
  float dt;
  float damping;
  float friction;
  float h;
  float tol;
  int bisect_it;
  float eps;
};

std::optional<Collision> point_triangle_ccd(
    const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config);

std::optional<Collision> edge_edge_ccd(
    const Eigen::Matrix<float, 3, 4>& position_t0,
    const Eigen::Matrix<float, 3, 4>& position_t1,
    const Eigen::Vector4f& weight, const CCDConfig& config);

}  // namespace silk
