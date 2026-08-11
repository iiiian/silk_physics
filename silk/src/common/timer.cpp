#include "common/timer.hpp"

#if SILK_ENABLE_TIMING

#include <spdlog/spdlog.h>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace silk {

using Clock = std::chrono::steady_clock;

struct TimingNode {
  std::string name;
  Clock::time_point start;
  double duration_ms = 0.0;
  std::vector<std::unique_ptr<TimingNode>> children;
};

namespace {

/// Global registry that stores the timer tree. Responsible for json output too.
class TimingRegistry {
 public:
  ~TimingRegistry() { write_json(); }

  TimingNode* begin(std::string_view name) {
    auto node = std::make_unique<TimingNode>();
    node->name = name;

    TimingNode* node_ptr = node.get();
    if (stack_.empty()) {
      roots_.push_back(std::move(node));
    } else {
      stack_.back()->children.push_back(std::move(node));
    }
    stack_.push_back(node_ptr);
    node_ptr->start = Clock::now();
    return node_ptr;
  }

  void end(TimingNode* node) {
    assert(!stack_.empty());
    assert(stack_.back() == node);

    Clock::time_point finish = Clock::now();
    node->duration_ms =
        std::chrono::duration<double, std::milli>(finish - node->start).count();
    stack_.pop_back();
  }

 private:
  static nlohmann::ordered_json to_json(const TimingNode& node) {
    nlohmann::ordered_json children = nlohmann::ordered_json::array();
    for (const std::unique_ptr<TimingNode>& child : node.children) {
      children.push_back(to_json(*child));
    }
    return {{"name", node.name},
            {"duration_ms", node.duration_ms},
            {"children", std::move(children)}};
  }

  void write_json() {
    nlohmann::ordered_json timings = nlohmann::ordered_json::array();
    for (const std::unique_ptr<TimingNode>& root : roots_) {
      timings.push_back(to_json(*root));
    }

    const char* configured_path = std::getenv("SILK_TIMINGS_PATH");
    std::string path = configured_path ? configured_path : "timings.json";
    std::ofstream output(path);
    if (!output) {
      std::cerr << "Failed to open timing output '" << path << "'.\n";
      return;
    }
    output << timings.dump(2) << '\n';
    if (!output) {
      std::cerr << "Failed to write timing output '" << path << "'.\n";
    }
  }

  std::vector<std::unique_ptr<TimingNode>> roots_;
  std::vector<TimingNode*> stack_;
};

TimingRegistry& timing_registry() {
  static TimingRegistry registry;
  return registry;
}

}  // namespace

Timer::Timer(std::string_view name) : node_(timing_registry().begin(name)) {}

Timer::~Timer() { end(); }

void Timer::end() {
  if (node_ == nullptr) {
    return;
  }

  TimingNode* node = node_;
  timing_registry().end(node);
  node_ = nullptr;

  SPDLOG_DEBUG("[timing] {}: {:.3f} ms", node->name, node->duration_ms);
}

}  // namespace silk

#endif
