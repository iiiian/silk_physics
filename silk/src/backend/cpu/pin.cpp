#include "backend/cpu/pin.hpp"

#include <Eigen/Core>
#include <cassert>
#include <span>

#include "common/eigen_alias.hpp"

namespace silk::cpu {

Pin::Pin(std::span<const int> index, const RMatrixX3f& V) {
  assert(index.size() <= V.rows());

  this->index.resize(index.size());
  this->curr_position.resize(3 * index.size());
  for (int pin_idx = 0; pin_idx < this->index.size(); ++pin_idx) {
    int i = index[pin_idx];
    assert(i >= 0 && i < V.rows());
    this->index[pin_idx] = i;
    curr_position(Eigen::seqN(3 * pin_idx, 3)) = V.row(i);
  }

  prev_position = curr_position;
}

}  // namespace silk::cpu
