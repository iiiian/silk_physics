#include "solver.hpp"

namespace silk {

PositionConstrain::PositionConstrain(Eigen::VectorXi vert_indexes,
                                     uint32_t offset, float weight)
    : vidx_(vert_indexes), offset_(offset), weight_(weight) {};

void PositionConstrain::project(const Eigen::VectorXf& verts,
                                Eigen::VectorXf& out) const {
  for (auto idx : vidx_) {
    auto seg = Eigen::seqN(offset_ + 3 * idx, 3);
    out(seg) = weight_ * verts(seg);
  }
}

}  // namespace silk
