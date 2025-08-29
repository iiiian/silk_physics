#pragma once

#include <Eigen/Core>

struct Pin {
  float pin_stiffness = 1e4f;

  Eigen::VectorXi index;
  Eigen::VectorXf position;
};
