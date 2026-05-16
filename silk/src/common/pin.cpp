#include "common/pin.hpp"

#include <Eigen/Core>
#include <cassert>
#include <span>

#include "common/eigen_alias.hpp"

namespace silk {

Pin::Pin(std::span<const int> index, const RMatrixX3f& V) {
  assert(index.size() < V.rows());

  this->index.resize(index.size());
  this->curr_position.resize(3 * index.size());
  for (int i : index) {
    assert(i >= 0 && i < V.rows());
    this->index[i] = i;
    curr_position(Eigen::seqN(3 * i, 3)) = V.row(i);
  }

  prev_position = curr_position;
}

}  // namespace silk
