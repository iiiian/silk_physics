#include <scalable_ccd/broad_phase/aabb.hpp>
#include <scalable_ccd/broad_phase/sort_and_sweep.hpp>

#include "cpu_adapters.hpp"

namespace silk::broadphase_benchmark {
namespace {

// Scalable CCD compares all three vertex ID slots, including unused slots.
// Give unused slots a negative ID derived from the first real vertex so
// unrelated primitives do not appear to share the common value -1.
std::array<long, 3> pad_vertex_ids(const Box& box) {
  std::array<long, 3> result = {box.vertex_ids[0], box.vertex_ids[1],
                                box.vertex_ids[2]};
  const long padding_id = -box.vertex_ids[0] - 1;
  for (long& vertex_id : result) {
    if (vertex_id < 0) {
      vertex_id = padding_id;
    }
  }
  return result;
}

scalable_ccd::AABB make_scalable_box(const Box& box) {
  scalable_ccd::AABB result(box.min.array(), box.max.array());
  result.vertex_ids = pad_vertex_ids(box);
  result.element_id = box.id;
  return result;
}

class ScalableSapAdapter final : public CpuAdapter {
 public:
  std::string name() const override { return "scalable_sap"; }

  void prepare(const QueryInput& input) override {
    input_ = input;
    boxes_a_.clear();
    boxes_a_.reserve(input.group_a.size());
    for (const Box& box : input.group_a) {
      boxes_a_.push_back(make_scalable_box(box));
    }
    boxes_b_.clear();
    boxes_b_.reserve(input.group_b.size());
    for (const Box& box : input.group_b) {
      boxes_b_.push_back(make_scalable_box(box));
    }
    output_.clear();
  }

  // Scalable CCD does not provide separate build and query APIs.
  void build() override {}

  void clear_output() override { output_.clear(); }

  void query() override {
    if (input_.kind == QueryKind::EE) {
      scalable_ccd::sort_and_sweep(boxes_a_, axis_, output_);
    } else {
      scalable_ccd::sort_and_sweep(boxes_a_, boxes_b_, axis_, output_);
    }
  }

  std::span<const Pair> output() const override { return output_; }

 private:
  QueryInput input_;
  int axis_ = 0;
  std::vector<scalable_ccd::AABB> boxes_a_;
  std::vector<scalable_ccd::AABB> boxes_b_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<CpuAdapter> make_scalable_sap_adapter() {
  return std::make_unique<ScalableSapAdapter>();
}

}  // namespace silk::broadphase_benchmark
