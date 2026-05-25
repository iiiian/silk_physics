#pragma once

#include <Eigen/Core>
#include <span>

#include "common/eigen_alias.hpp"

namespace silk::cpu {

class Pin {
 public:
  bool is_static = true;        //< True if remain static for >=1 frame.
  bool is_static_twice = true;  //< True if remain static for >=2 frame.
  bool is_all_pinned = false;   //< True if all vertices are pinned.
  Eigen::VectorXi index;        //< Pinned indices. Useful if not is_all_pinned.
  Eigen::VectorXf curr_position;  //< Current vertex position.
  Eigen::VectorXf prev_position;  //< Previous vertex position.

  Pin() = default;
  Pin(std::span<const int> index, const RMatrixX3f& V);
};

}  // namespace silk::cpu
