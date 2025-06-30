#pragma once

namespace silk {

class Interval {
  bool is_empty_;
  float lower_;
  float upper_;

 public:
  Interval();
  Interval(float a, float b);

  bool is_empty() const;
  float lower_bound() const;
  float upper_bound() const;
  Interval intersection(const Interval& other) const;
};

}  // namespace silk
