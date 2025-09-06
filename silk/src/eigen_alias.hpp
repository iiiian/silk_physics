#pragma once

#include <Eigen/Core>

namespace silk {

using RMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RMatrixX2i = Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor>;

}  // namespace silk
