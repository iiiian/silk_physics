#pragma once

#include <Eigen/Core>

struct Pin {
  float pin_sitffness = 1e6f;

  Eigen::VectorXf index;
  Eigen::VectorXf value;
};
