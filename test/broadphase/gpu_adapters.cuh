#pragma once

#include <memory>
#include <string>

#include "broadphase_common.hpp"

namespace silk::broadphase_benchmark {

class GpuAdapter {
 public:
  virtual ~GpuAdapter() = default;

  virtual std::string name() const = 0;
  /// Convert current-frame inputs to the adapter's host format. Not timed.
  virtual void prepare(const QueryInput& input) = 0;
  /// Upload inputs and build or update the broadphase.
  virtual void build() = 0;
  /// Reset query output. Not timed.
  virtual void clear_output() = 0;
  /// Run the complete native output protocol, including required allocation,
  /// retries, and D2H result transfer.
  virtual void query() = 0;
  virtual void synchronize() = 0;
  /// Convert downloaded native output to benchmark pairs. Not timed.
  virtual void materialize_output() = 0;
  virtual std::span<const Pair> output() const = 0;
};

std::unique_ptr<GpuAdapter> make_silk_oibvh_adapter();
std::unique_ptr<GpuAdapter> make_scalable_stq_adapter();
std::unique_ptr<GpuAdapter> make_cubql_adapter();

}  // namespace silk::broadphase_benchmark
