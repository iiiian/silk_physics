#include "backend/cuda/collision/interval.hpp"

namespace silk::cuda {

Interval::Interval(float lower, float upper) : lower(lower), upper(upper) {}

std::pair<Interval, Interval> Interval::bisect() const {
  // Use the arithmetic midpoint; sufficient for float ranges here.
  float mid = (upper + lower) / 2;
  return std::make_pair(Interval(lower, mid), Interval(mid, upper));
}

}  // namespace silk::cuda
