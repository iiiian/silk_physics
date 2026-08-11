#pragma once

#include <string_view>

#ifndef SILK_ENABLE_TIMING
#define SILK_ENABLE_TIMING 0
#endif

namespace silk {

#if SILK_ENABLE_TIMING

struct TimingNode;

/// @brief Scoped hierarchical timer with structured json output.
///
/// Example.
/// Timer timer("Collision Detection"); // Start the timer
/// // do computation
/// timer.end(); // Optional early end. Else it stop at scope end.
class Timer {
 public:
  explicit Timer(std::string_view name);
  ~Timer();

  Timer(const Timer&) = delete;
  Timer(Timer&&) = delete;
  Timer& operator=(const Timer&) = delete;
  Timer& operator=(Timer&&) = delete;

  /// @brief End the current timer.
  void end();

 private:
  TimingNode* node_ = nullptr;
};

#else

/// @brief No-op timer used when timing is disabled.
class Timer {
 public:
  explicit Timer(std::string_view) noexcept {}

  Timer(const Timer&) = delete;
  Timer(Timer&&) = delete;
  Timer& operator=(const Timer&) = delete;
  Timer& operator=(Timer&&) = delete;

  void end() noexcept {}
};

#endif

}  // namespace silk
