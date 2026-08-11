#include "common/timer.hpp"

#if SILK_ENABLE_TIMING

#ifdef SILK_WITH_CUDA
#include "backend/cuda/cuda_utils.cuh"
#endif

#include <spdlog/spdlog.h>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
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
#ifdef SILK_WITH_CUDA
  std::optional<::cuda::stream_ref> cuda_stream;
  std::optional<::cuda::timed_event> cuda_start;
#endif
};

namespace {

/// Global registry that stores the timer tree. Responsible for json output too.
class TimingRegistry {
 public:
  ~TimingRegistry() { write_json(); }

  TimingNode* begin(std::string_view name) {
    auto node = std::make_unique<TimingNode>();
    node->name = name;
    node->start = Clock::now();
    return add(std::move(node));
  }

  void end(TimingNode* node) {
    assert(!stack_.empty());
    assert(stack_.back() == node);

    Clock::time_point finish = Clock::now();
    node->duration_ms =
        std::chrono::duration<double, std::milli>(finish - node->start).count();
    stack_.pop_back();
  }

#ifdef SILK_WITH_CUDA
  TimingNode* begin(std::string_view name, ::cuda::stream_ref stream) {
    auto node = std::make_unique<TimingNode>();
    node->name = name;
    node->cuda_stream = stream;
    node->cuda_start.emplace(stream);
    return add(std::move(node));
  }

  void end_cuda(TimingNode* node) {
    assert(!stack_.empty());
    assert(stack_.back() == node);
    assert(node->cuda_stream);
    assert(node->cuda_start);

    ::cuda::timed_event stop(*node->cuda_stream);
    node->cuda_stream->sync();

    auto duration = stop - *node->cuda_start;
    node->duration_ms = static_cast<double>(duration.count()) / 1'000'000.0;
    node->cuda_start.reset();
    node->cuda_stream.reset();
    stack_.pop_back();
  }
#endif

 private:
  TimingNode* add(std::unique_ptr<TimingNode> node) {
    TimingNode* node_ptr = node.get();
    if (stack_.empty()) {
      roots_.push_back(std::move(node));
    } else {
      stack_.back()->children.push_back(std::move(node));
    }
    stack_.push_back(node_ptr);
    return node_ptr;
  }

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

#ifdef SILK_WITH_CUDA
Timer::Timer(std::string_view name, cuda::CudaRuntime rt)
    : node_(timing_registry().begin(name, rt.stream)) {}
#endif

Timer::~Timer() { end(); }

void Timer::end() {
  if (node_ == nullptr) {
    return;
  }

  TimingNode* node = node_;
#ifdef SILK_WITH_CUDA
  if (node->cuda_start) {
    timing_registry().end_cuda(node);
  } else {
    timing_registry().end(node);
  }
#else
  timing_registry().end(node);
#endif
  node_ = nullptr;

  SPDLOG_DEBUG("[timing] {}: {:.3f} ms", node->name, node->duration_ms);
}

}  // namespace silk

#endif
