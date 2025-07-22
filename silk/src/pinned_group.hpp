#pragma once

#include <Eigen/Core>

struct PinnedGroup {
  Eigen::VectorXf pinnned_index;
  Eigen::VectorXf pinned_value;
  Eigen::VectorXf prev_pinned_value;
};
