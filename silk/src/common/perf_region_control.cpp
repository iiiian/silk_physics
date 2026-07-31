#include "common/perf_region_control.hpp"

#include <unistd.h>

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <string>

namespace silk {

PerfRegionControl::PerfRegionControl() {
  // Get control/act file descriptor.
  const char *ctl = std::getenv("PERF_CTL_FD");
  const char *ack = std::getenv("PERF_ACK_FD");
  if (!ctl || !ack) {
    return;
  }
  ctl_fd_ = std::stoi(ctl);
  ack_fd_ = std::stoi(ack);
  active_ = true;
}

void PerfRegionControl::enable() { command("enable"); }

void PerfRegionControl::disable() { command("disable"); }

void PerfRegionControl::command(const char *text) {
  if (!active_) {
    return;
  }
  std::string command_text(text);
  command_text += '\n';
  ssize_t n = write(ctl_fd_, command_text.data(), command_text.size());
  assert(n == static_cast<ssize_t>(command_text.size()));

  char ack[4] = {};
  ssize_t got = read(ack_fd_, ack, sizeof(ack));
  assert(got == 4 && std::memcmp(ack, "ack\n", 4) == 0);
}

}  // namespace silk
