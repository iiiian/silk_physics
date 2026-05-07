#pragma once

#include <Eigen/Core>

namespace silk {

struct Pin {
  bool is_static;         //< True if remain static for >=1 frame.
  bool is_static_twice;   //< True if remain static for >=2 frame.
  bool is_all_pinned;     //< True if all vertices are pinned.
  Eigen::VectorXi index;  //< Pinned indices. Useful if not is_all_pinned.
  Eigen::VectorXf curr_position;  //< Current vertex position.
  Eigen::VectorXf prev_position;  //< Previous vertex position.
};

}  // namespace silk
