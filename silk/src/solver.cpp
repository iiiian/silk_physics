#include "solver.hpp"

namespace silk {

PositionConstrain::PositionConstrain(Eigen::VectorXi vert_indexes, int offset,
                                     float weight)
    : vert_indexes_(vert_indexes), offset_(offset), weight_(weight) {};

void PositionConstrain::project(const Eigen::VectorXf& position,
                                Eigen::VectorXf& out) const {
  for (int idx : vert_indexes_) {
    auto seg = Eigen::seqN(offset_ + 3 * idx, 3);
    out(seg) += weight_ * position(seg);
  }
}

}  // namespace silk
