#include "interval.hpp"

#include <algorithm>

namespace silk {

Interval::Interval() {
  is_empty_ = true;
  lower_ = 0;
  upper_ = 0;
};

Interval::Interval(float a, float b) {
  if (a < b) {
    is_empty_ = false;
    lower_ = a;
    upper_ = b;
  } else if (a == b) {
    Interval();
  } else {
    is_empty_ = false;
    lower_ = b;
    upper_ = a;
  }
}

bool Interval::is_empty() const { return is_empty_; }

float Interval::lower_bound() const { return lower_; }

float Interval::upper_bound() const { return upper_; }

Interval Interval::intersection(const Interval& other) const {
  if (is_empty_ || other.is_empty_) {
    return Interval();
  }

  float new_lower = std::max(lower_, other.lower_);
  float new_upper = std::min(upper_, other.upper_);
  if (new_lower >= new_upper) {
    return Interval();
  }
  return Interval(new_lower, new_upper);
}

}  // namespace silk
