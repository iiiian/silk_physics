#pragma once

#include <Eigen/Core>

using Vert = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Face = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Edge = Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor>;
