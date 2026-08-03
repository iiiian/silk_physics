#include <tbb/parallel_for.h>

#include <cuda/devices>
#include <cuda/memory_pool>
#include <cuda/stream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "backend/cpu/collision/ccd.hpp"
#include "backend/cuda/collision/ccd_fast_rejection.cuh"
#include "backend/cuda/cuda_utils.cuh"
#include "narrowphase_common.hpp"

namespace silk::narrowphase_benchmark {

namespace {

constexpr float TOLERANCE = 1e-6f;

cuda::Vec3f make_cuda_vector(const Eigen::Vector3f& value) {
  return cuda::Vec3f::vec_like(value);
}

cuda::CCDQuery make_cuda_query(const Query& input, QueryKind kind,
                               int source_index) {
  cuda::CCDQuery query{};
  query.source_index = source_index;
  query.minimal_separation = compute_minimum_separation(input, kind);
  for (int i = 0; i < 4; ++i) {
    query.position_t0[i] = make_cuda_vector(input.vertices[i]);
    query.position_t1[i] = make_cuda_vector(input.vertices[i + 4]);
  }
  return query;
}

QueryOutput solve_query(const Query& input, QueryKind kind,
                        const Eigen::Array3f& numerical_error,
                        int max_iterations) {
  std::optional<cpu::CCDResult> result;
  float minimum_separation = compute_minimum_separation(input, kind);
  if (kind == QueryKind::VF) {
    result = cpu::vertex_face_ccd(
        input.vertices[0], input.vertices[1], input.vertices[2],
        input.vertices[3], input.vertices[4], input.vertices[5],
        input.vertices[6], input.vertices[7], numerical_error,
        minimum_separation, TOLERANCE, max_iterations, false);
  } else {
    result = cpu::edge_edge_ccd(
        input.vertices[0], input.vertices[1], input.vertices[2],
        input.vertices[3], input.vertices[4], input.vertices[5],
        input.vertices[6], input.vertices[7], numerical_error,
        minimum_separation, TOLERANCE, max_iterations, false);
  }
  return {.toi = result ? result->t(0) : 1.0f, .hit = result.has_value()};
}

class SilkHybridTiccdAdapter final : public GpuAdapter {
 public:
  SilkHybridTiccdAdapter(QueryKind kind, int max_iterations)
      : kind_(kind),
        max_iterations_(max_iterations),
        device_(::cuda::devices[0]),
        stream_(device_),
        memory_pool_(device_) {}

  std::string name() const override { return "silk_hybrid_ticcd"; }

  void prepare(std::span<const Query> queries) override {
    input_ = queries;
    device_input_.clear();
    device_input_.reserve(queries.size());
    for (int i = 0; i < queries.size(); ++i) {
      device_input_.push_back(make_cuda_query(queries[i], kind_, i));
    }
    output_.assign(queries.size(), {});
    if (!queries.empty()) {
      Eigen::Vector3f error = queries.front().err.matrix();
      err_ = make_cuda_vector(error);
      device_queries_ =
          cuda::vec_like_to_device<cuda::CCDQuery>(device_input_, runtime());
    } else {
      device_queries_.reset();
    }
  }

  void synchronize() override { stream_.sync(); }

  void query() override {
    if (input_.empty()) {
      return;
    }

    std::vector<cuda::CCDQuery> unresolved = cuda::ticcd_rejection(
        ::cuda::std::span<const cuda::CCDQuery>(device_queries_->data(),
                                                device_queries_->size()),
        kind_ == QueryKind::VF, err_, runtime());
    Eigen::Array3f numerical_error{err_(0), err_(1), err_(2)};
    int unresolved_num = unresolved.size();
    tbb::parallel_for(0, unresolved_num, [&](int i) {
      int source_index = unresolved[i].source_index;
      if (source_index < 0 || source_index >= output_.size()) {
        throw std::runtime_error(
            "Silk hybrid TICCD returned an invalid source query");
      }
      output_[source_index] = solve_query(input_[source_index], kind_,
                                          numerical_error, max_iterations_);
    });
  }

  std::span<const QueryOutput> output() const override { return output_; }

 private:
  cuda::CudaRuntime runtime() {
    return {.stream = stream_, .mr = memory_pool_.as_ref()};
  }

  QueryKind kind_;
  int max_iterations_;
  ::cuda::device_ref device_;
  ::cuda::stream stream_;
  ::cuda::device_memory_pool memory_pool_;
  cuda::Vec3f err_ = cuda::Vec3f::zeros();
  std::span<const Query> input_;
  std::vector<cuda::CCDQuery> device_input_;
  cuda::Buf<cuda::CCDQuery> device_queries_;
  std::vector<QueryOutput> output_;
};

}  // namespace

std::unique_ptr<GpuAdapter> make_silk_hybrid_ticcd_adapter(QueryKind kind,
                                                           int max_iterations) {
  return std::make_unique<SilkHybridTiccdAdapter>(kind, max_iterations);
}

}  // namespace silk::narrowphase_benchmark
