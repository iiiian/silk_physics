#include "interval.hpp"

#include <algorithm>

namespace silk {

Interval::Interval() {
  is_empty_ = true;
  lb_ = 0;
  ub_ = 0;
};

Interval::Interval(float a, float b) {
  if (a < b) {
    is_empty_ = false;
    lb_ = a;
    ub_ = b;
  } else if (a == b) {
    Interval();
  } else {
    is_empty_ = false;
    lb_ = b;
    ub_ = a;
  }
}

bool Interval::is_empty() const { return is_empty_; }

float Interval::lower_bound() const { return lb_; }

float Interval::upper_bound() const { return ub_; }

Interval Interval::intersection(const Interval& other) const {
  if (is_empty_ || other.is_empty_) {
    return Interval();
  }

  float new_lb = std::max(lb_, other.lb_);
  float new_ub = std::min(ub_, other.ub_);
  if (new_lb >= new_ub) {
    return Interval();
  }
  return Interval(new_lb, new_ub);
}

}  // namespace silk
