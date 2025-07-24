#pragma once

#include <Eigen/Core>

namespace silk {

struct ObstaclePosition {
  bool is_moving;
  Eigen::VectorXf position;
  Eigen::VectorXf prev_position;
};

}  // namespace silk
