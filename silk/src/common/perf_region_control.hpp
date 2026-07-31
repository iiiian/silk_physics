#pragma once

namespace silk {

/// @brief Helper class for perf ROI profiling.
///
/// PerfRegionControl roi_control;
/// roi_control.enable();
/// do_something();
/// roi_control.disable();
class PerfRegionControl {
 public:
  PerfRegionControl();

  void enable();
  void disable();

 private:
  // Write command to control file, then confirm ack.
  void command(const char *text);

  int ctl_fd_ = -1;
  int ack_fd_ = -1;
  bool active_ = false;
};

}  // namespace silk
