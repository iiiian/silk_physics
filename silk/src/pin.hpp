#pragma once

#include <Eigen/Core>

struct Pin {
  float pin_stiffness = 1e10f;

  Eigen::VectorXi index;
  Eigen::VectorXf position;
};
