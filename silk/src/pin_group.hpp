#pragma once

#include <Eigen/Core>

struct PinGroup {
  float pin_sitffness = 1e6f;

  Eigen::VectorXf pin_index;
  Eigen::VectorXf pin_value;
  Eigen::VectorXf prev_pin_value;
};
