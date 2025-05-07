#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <optional>

using Matrix69f = Eigen::Matrix<float, 6, 9>;

// return vectorize jacobian from 2d reference config to 3d deformed config
// return nullopt if triangle is degenerated
std::optional<Matrix69f> vectorized_jacobian(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2);
