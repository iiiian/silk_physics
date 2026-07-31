#pragma once

#include <memory>
#include <string>

#include "broadphase_common.hpp"

namespace silk::broadphase_benchmark {

class CpuAdapter {
 public:
  virtual ~CpuAdapter() = default;

  virtual std::string name() const = 0;
  /// Convert current-frame inputs without resetting acceleration state. This is
  /// not included in the timings.
  virtual void prepare(const QueryInput& input) = 0;
  /// Build or update the broadphase data structure, including root bounds.
  virtual void build() = 0;
  /// Clear query output. This is not included in the query timing.
  virtual void clear_output() = 0;
  /// Run broadphase collision query, including required thread-buffer merges.
  virtual void query() = 0;
  /// Convert native query output to benchmark pairs. Not timed.
  virtual void materialize_output() {}
  /// Get query output.
  virtual std::span<const Pair> output() const = 0;
};

std::unique_ptr<CpuAdapter> make_silk_kdtree_adapter();
std::unique_ptr<CpuAdapter> make_scalable_sap_adapter();
std::unique_ptr<CpuAdapter> make_embree_adapter();

}  // namespace silk::broadphase_benchmark
