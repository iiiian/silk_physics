#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

Eigen::Matrix<float, 6, 9> vectorized_F_operator(
    Eigen::Ref<const Eigen::Vector3f> v0, Eigen::Ref<const Eigen::Vector3f> v1,
    Eigen::Ref<const Eigen::Vector3f> v2);
