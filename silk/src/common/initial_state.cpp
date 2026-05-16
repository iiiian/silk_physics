#include "common/initial_state.hpp"

#include <Eigen/Core>

#include "common/eigen_alias.hpp"

namespace silk {

InitialState::InitialState(const RMatrixX3f& V) {
  position = V.reshaped<Eigen::RowMajor>();
  velocity.resizeLike(position);
}

}  // namespace silk
