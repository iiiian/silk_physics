#pragma once

#include <Eigen/Core>

namespace silk::cuda {

class ObjectState {
 public:
  // The range of this object in the global state array.
  int state_offset = 0;
  int state_num = 0;

  float* d_curr_state = nullptr;
  float* d_state_velocity = nullptr;

 public:
  ObjectState() = default;
  ObjectState(int state_offset, const Eigen::VectorXf& curr_state,
              const Eigen::VectorXf& state_velocity);
  ObjectState(const ObjectState& other) = delete;
  ObjectState(ObjectState&& other) noexcept;
  ObjectState& operator=(const ObjectState& other) = delete;
  ObjectState& operator=(ObjectState&& other) noexcept;
  ~ObjectState();

 private:
  void swap(ObjectState& other) noexcept;
};

}  // namespace silk::cuda
