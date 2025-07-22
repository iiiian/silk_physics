#pragma once

#include <Eigen/Core>

struct PinnedGroup {
  float pinned_sitffness = 1e6f;

  Eigen::VectorXf pinnned_index;
  Eigen::VectorXf pinned_value;
  Eigen::VectorXf prev_pinned_value;
};
