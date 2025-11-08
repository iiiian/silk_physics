#prama once

#include <Eigen/Core>

namespace silk {

class GpuObjectState {
 public:
  int state_offset = 0;
  int state_num = 0;

  float* d_curr_state = nullptr;
  float* d_state_velocity = nullptr;

 public:
  GpuObjectState() = default;
  GpuObjectState(const GpuObjectState& other) = delete;
  GpuObjectState(GpuObjectState&& other) = default;

  GpuObjectState& operator=(const GpuObjectState& other) = delete;
  GpuObjectState& operator=(GpuObjectState&& other) = default;

  ~GpuObjectState();

  Eigen::VectorXf curr_state_to_host() const;
  void curr_state_to_device(Eigen::Ref<const Eigen::VectorXf> h_current_state);
  Eigen::VectorXf state_velocity_to_host() const;
  void state_velocity_to_device(
      Eigen::Ref<const Eigen::VectorXf> h_state_velocity);
}

}  // namespace silk
