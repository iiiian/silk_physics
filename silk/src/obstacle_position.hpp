#pragma once

#include <Eigen/Core>

namespace silk {

struct ObstaclePosition {
  bool is_static;        // True if obstacle is static for 1+ steps.
  bool is_static_twice;  // True if obstacle is static for 2+ steps.
  Eigen::VectorXf curr_position;
  Eigen::VectorXf prev_position;  // Valid only if obstacle is not static.
};

}  // namespace silk
