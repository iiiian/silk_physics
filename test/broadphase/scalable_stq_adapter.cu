#include <memory>
#include <scalable_ccd/cuda/broad_phase/aabb.cuh>
#include <scalable_ccd/cuda/broad_phase/broad_phase.cuh>
#include <vector>

#include "gpu_adapter_common.cuh"

namespace silk::broadphase_benchmark {
namespace {

// Scalable CCD compares all three vertex ID slots, including unused slots.
// Give unused slots a unique negative ID derived from the first real vertex so
// unrelated primitives do not appear to share the common value -1.
std::array<int, 3> pad_vertex_ids(const Box& box) {
  std::array<int, 3> result = box.vertex_ids;
  const int padding_id = -box.vertex_ids[0] - 1;
  for (int& vertex_id : result) {
    if (vertex_id < 0) {
      vertex_id = padding_id;
    }
  }
  return result;
}

scalable_ccd::cuda::AABB make_scalable_gpu_box(const Box& box) {
  using scalable_ccd::cuda::make_Scalar3;
  scalable_ccd::cuda::AABB result(
      make_Scalar3(box.min.x(), box.min.y(), box.min.z()),
      make_Scalar3(box.max.x(), box.max.y(), box.max.z()));
  const std::array<int, 3> vertex_ids = pad_vertex_ids(box);
  result.vertex_ids = make_int3(vertex_ids[0], vertex_ids[1], vertex_ids[2]);
  result.element_id = box.id;
  return result;
}

std::vector<scalable_ccd::cuda::AABB> make_scalable_gpu_boxes(
    std::span<const Box> boxes) {
  std::vector<scalable_ccd::cuda::AABB> result;
  result.reserve(boxes.size());
  for (const Box& box : boxes) {
    result.push_back(make_scalable_gpu_box(box));
  }
  return result;
}

class ScalableStqAdapter final : public GpuAdapter {
 public:
  std::string name() const override { return "scalable_stq"; }

  void prepare(const QueryInput& input) override {
    kind_ = input.kind;
    host_a_ = make_scalable_gpu_boxes(input.group_a);
    host_b_ = make_scalable_gpu_boxes(input.group_b);
    output_.clear();
  }

  void build() override {
    auto boxes_a = std::make_shared<scalable_ccd::cuda::DeviceAABBs>(host_a_);
    if (kind_ == QueryKind::EE) {
      broadphase_.build(boxes_a);
    } else {
      auto boxes_b = std::make_shared<scalable_ccd::cuda::DeviceAABBs>(host_b_);
      broadphase_.build(boxes_a, boxes_b);
    }
  }

  void clear_output() override { output_.clear(); }

  void query() override { output_ = broadphase_.detect_overlaps(); }

  void synchronize() override { CHECK_CUDA(cudaDeviceSynchronize()); }

  void materialize_output() override {}

  std::span<const Pair> output() const override { return output_; }

 private:
  QueryKind kind_ = QueryKind::EE;
  std::vector<scalable_ccd::cuda::AABB> host_a_;
  std::vector<scalable_ccd::cuda::AABB> host_b_;
  scalable_ccd::cuda::BroadPhase broadphase_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<GpuAdapter> make_scalable_stq_adapter() {
  return std::make_unique<ScalableStqAdapter>();
}

}  // namespace silk::broadphase_benchmark
