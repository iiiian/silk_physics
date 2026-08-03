#include <cuda_runtime.h>

#include <algorithm>
#include <ccdgpu/Type.hpp>
#include <ccdgpu/record.hpp>
#include <ccdgpu/root_finder.cuh>
#include <limits>
#include <memory>
#include <stdexcept>
#include <stq/gpu/memory.cuh>
#include <string>
#include <vector>

#include "narrowphase_common.hpp"

namespace silk::narrowphase_benchmark {

namespace {

constexpr ccd::Scalar TOLERANCE = 1e-6f;
constexpr ccd::Scalar NO_COLLISION_TOI = 2;
constexpr int MAX_ITERATIONS = -1;
constexpr int CUDA_THREADS = 64;

void check_cuda(cudaError_t error) {
  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }
}

float parse_toi(const nlohmann::json& value) {
  long double numerator = std::stold(value.at(2).get<std::string>());
  long double denominator = std::stold(value.at(3).get<std::string>());
  if (denominator == 0) {
    throw std::runtime_error("Scalable CCD returned a zero TOI denominator");
  }
  return static_cast<float>(numerator / denominator);
}

class ScalableGpuAdapterImpl final : public ScalableGpuAdapter {
 public:
  explicit ScalableGpuAdapterImpl(QueryKind kind) : kind_(kind) {}

  std::string name() const override { return "scalable_ccd"; }

  void prepare(std::span<const Query> queries) override {
    if (queries.size() > std::numeric_limits<int>::max()) {
      throw std::runtime_error("Too many Scalable CCD queries");
    }

    input_.resize(queries.size());
    outputs_.assign(queries.size(), {});
    for (int query_index = 0; query_index < queries.size(); ++query_index) {
      ccd::CCDData& data = input_[query_index];
      const Query& query = queries[query_index];
      for (int axis = 0; axis < 3; ++axis) {
        data.v0s[axis] = query.vertices[0][axis];
        data.v1s[axis] = query.vertices[1][axis];
        data.v2s[axis] = query.vertices[2][axis];
        data.v3s[axis] = query.vertices[3][axis];
        data.v0e[axis] = query.vertices[4][axis];
        data.v1e[axis] = query.vertices[5][axis];
        data.v2e[axis] = query.vertices[6][axis];
        data.v3e[axis] = query.vertices[7][axis];
      }
      data.ms = compute_minimum_separation(query, kind_);
      data.toi = NO_COLLISION_TOI;
      data.aid = query_index;
      data.bid = query_index;
      data.nbr_checks = 0;
    }
  }

  void synchronize() override { check_cuda(cudaDeviceSynchronize()); }

  void query() override {
    outputs_.assign(input_.size(), {});
    stq::gpu::MemHandler memory_handler;
    memory_handler.MAX_QUERIES = input_.size();

    int start = 0;
    while (start < input_.size()) {
      const int remaining = input_.size() - start;
      int query_num = std::min<int>(remaining, memory_handler.MAX_QUERIES);
      bool overflowed = false;
      ccd::gpu::Record record;

      do {
        if (!overflowed) {
          memory_handler.handleNarrowPhase(query_num);
          memory_handler.handleOverflow(query_num);
        } else {
          memory_handler.handleOverflow(query_num);
        }
        if (memory_handler.MAX_UNIT_SIZE % 2 == 0) {
          ++memory_handler.MAX_UNIT_SIZE;
        }
        if (query_num <= 0) {
          throw std::runtime_error("Insufficient GPU memory for Scalable CCD");
        }

        ccd::CCDData* device_data = nullptr;
        check_cuda(cudaMalloc(&device_data, sizeof(ccd::CCDData) * query_num));
        check_cuda(cudaMemcpy(device_data, input_.data() + start,
                              sizeof(ccd::CCDData) * query_num,
                              cudaMemcpyHostToDevice));

        int overflow = 0;
        ccd::Scalar earliest_toi = 1;
        std::vector<int> unused_results;
        record.Clear();
        ccd::run_memory_pool_ccd(device_data, &memory_handler, query_num,
                                 kind_ == QueryKind::EE, unused_results,
                                 CUDA_THREADS, MAX_ITERATIONS, TOLERANCE, true,
                                 true, earliest_toi, overflow, record);
        overflowed = overflow != 0;
      } while (overflowed);

      if (record.j_object.contains("toi_per_query")) {
        for (const nlohmann::json& collision :
             record.j_object.at("toi_per_query")) {
          int query_index = std::stoi(collision.at(0).get<std::string>());
          if (query_index < start || query_index >= start + query_num) {
            throw std::runtime_error(
                "Scalable CCD returned an invalid query ID");
          }
          outputs_[query_index] = {.toi = parse_toi(collision), .hit = true};
        }
      }
      start += query_num;
    }
  }

  std::span<const QueryOutput> output() const override { return outputs_; }

 private:
  QueryKind kind_;
  std::vector<ccd::CCDData> input_;
  std::vector<QueryOutput> outputs_;
};

}  // namespace

std::unique_ptr<ScalableGpuAdapter> make_scalable_gpu_adapter(QueryKind kind) {
  return std::make_unique<ScalableGpuAdapterImpl>(kind);
}

}  // namespace silk::narrowphase_benchmark
