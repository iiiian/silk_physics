#include <cuBQL/bvh.h>

// cuBQL's CUDA builder header requires the BVH declarations first.
#include <cuBQL/builder/cuda.h>
#include <cuBQL/traversal/fixedBoxQuery.h>
#include <thrust/device_vector.h>

#include <vector>

#include "gpu_adapter_common.cuh"

namespace silk::broadphase_benchmark {
namespace {

struct CubqlMetadata {
  int vertex_ids[3];
};

__device__ bool cubql_overlap(const cuBQL::box3f& a, const cuBQL::box3f& b) {
  return a.lower.x <= b.upper.x && b.lower.x <= a.upper.x &&
         a.lower.y <= b.upper.y && b.lower.y <= a.upper.y &&
         a.lower.z <= b.upper.z && b.lower.z <= a.upper.z;
}

__device__ bool cubql_share_vertex(const CubqlMetadata& a,
                                   const CubqlMetadata& b) {
  for (int i = 0; i < 3; ++i) {
    if (a.vertex_ids[i] < 0) {
      continue;
    }
    for (int j = 0; j < 3; ++j) {
      if (a.vertex_ids[i] == b.vertex_ids[j]) {
        return true;
      }
    }
  }
  return false;
}

// clang-format off
__global__ void cubql_query_kernel(
    const cuBQL::box3f* query_boxes,
    const CubqlMetadata* query_metadata,
    int query_num,
    const cuBQL::box3f* target_boxes,
    const CubqlMetadata* target_metadata,
    cuBQL::bvh3f bvh,
    bool self_query,
    int2* output,
    int output_capacity,
    int* output_num)
// clang-format on
{
  int query_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_id >= query_num) {
    return;
  }
  cuBQL::fixedBoxQuery::forEachPrim(
      [=] __device__(int target_id) {
        if (self_query && target_id <= query_id) {
          return CUBQL_CONTINUE_TRAVERSAL;
        }
        if (cubql_share_vertex(query_metadata[query_id],
                               target_metadata[target_id])) {
          return CUBQL_CONTINUE_TRAVERSAL;
        }
        if (!cubql_overlap(query_boxes[query_id], target_boxes[target_id])) {
          return CUBQL_CONTINUE_TRAVERSAL;
        }

        int output_id = atomicAdd(output_num, 1);
        if (output_id < output_capacity) {
          int2 pair = self_query ? make_int2(query_id, target_id)
                                 : make_int2(target_id, query_id);
          output[output_id] = pair;
        }
        return CUBQL_CONTINUE_TRAVERSAL;
      },
      bvh, query_boxes[query_id]);
}

class CubqlAdapter final : public GpuAdapter {
 public:
  CubqlAdapter() { CHECK_CUDA(cudaStreamCreate(&stream_)); }

  ~CubqlAdapter() override {
    if (bvh_.nodes != nullptr) {
      cuBQL::cuda::free(bvh_, stream_);
      cudaStreamSynchronize(stream_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  std::string name() const override { return "cubql"; }

  void prepare(const QueryInput& input) override {
    input_ = input;
    make_host_boxes(input.group_a, host_boxes_a_, host_metadata_a_);
    make_host_boxes(input.group_b, host_boxes_b_, host_metadata_b_);
    output_.clear();
    native_output_.clear();
  }

  void build() override {
    upload_boxes(host_boxes_a_, host_metadata_a_, boxes_a_, metadata_a_,
                 stream_);
    upload_boxes(host_boxes_b_, host_metadata_b_, boxes_b_, metadata_b_,
                 stream_);
    if (output_num_.empty()) {
      output_num_.resize(1);
    }
    if (bvh_.nodes != nullptr) {
      cuBQL::cuda::free(bvh_, stream_);
      bvh_ = {};
    }
    cuBQL::BuildConfig config;
    config.makeLeafThreshold = 4;
    cuBQL::cuda::radixBuilder(bvh_, thrust::raw_pointer_cast(boxes_a_.data()),
                              boxes_a_.size(), config, stream_);
  }

  void clear_output() override {
    output_.clear();
    native_output_.clear();
    CHECK_CUDA(cudaMemsetAsync(thrust::raw_pointer_cast(output_num_.data()), 0,
                               sizeof(int), stream_));
  }

  void query() override {
    query_device(output_device_.empty()
                     ? nullptr
                     : thrust::raw_pointer_cast(output_device_.data()),
                 output_device_.size());
    int count = 0;
    CHECK_CUDA(cudaMemcpyAsync(&count,
                               thrust::raw_pointer_cast(output_num_.data()),
                               sizeof(int), cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    if (count > output_device_.size()) {
      output_device_.resize(count);
      CHECK_CUDA(cudaMemsetAsync(thrust::raw_pointer_cast(output_num_.data()),
                                 0, sizeof(int), stream_));
      query_device(thrust::raw_pointer_cast(output_device_.data()),
                   output_device_.size());
    }

    native_output_.resize(count);
    if (count > 0) {
      CHECK_CUDA(cudaMemcpyAsync(
          native_output_.data(),
          thrust::raw_pointer_cast(output_device_.data()), count * sizeof(int2),
          cudaMemcpyDeviceToHost, stream_));
    }
    CHECK_CUDA(cudaStreamSynchronize(stream_));
  }

  void synchronize() override { CHECK_CUDA(cudaStreamSynchronize(stream_)); }

  void materialize_output() override {
    output_.reserve(native_output_.size());
    for (const int2 pair : native_output_) {
      output_.emplace_back(pair.x, pair.y);
    }
  }

  std::span<const Pair> output() const override { return output_; }

 private:
  static void make_host_boxes(std::span<const Box> input,
                              std::vector<cuBQL::box3f>& boxes,
                              std::vector<CubqlMetadata>& metadata) {
    boxes.clear();
    metadata.clear();
    boxes.reserve(input.size());
    metadata.reserve(input.size());
    for (const Box& box : input) {
      boxes.emplace_back(cuBQL::vec3f(box.min.x(), box.min.y(), box.min.z()),
                         cuBQL::vec3f(box.max.x(), box.max.y(), box.max.z()));
      metadata.push_back(
          {{box.vertex_ids[0], box.vertex_ids[1], box.vertex_ids[2]}});
    }
  }

  static void upload_boxes(const std::vector<cuBQL::box3f>& host_boxes,
                           const std::vector<CubqlMetadata>& host_metadata,
                           thrust::device_vector<cuBQL::box3f>& boxes,
                           thrust::device_vector<CubqlMetadata>& metadata,
                           cudaStream_t stream) {
    boxes.resize(host_boxes.size());
    metadata.resize(host_metadata.size());
    if (!host_boxes.empty()) {
      CHECK_CUDA(cudaMemcpyAsync(thrust::raw_pointer_cast(boxes.data()),
                                 host_boxes.data(),
                                 host_boxes.size() * sizeof(cuBQL::box3f),
                                 cudaMemcpyHostToDevice, stream));
      CHECK_CUDA(cudaMemcpyAsync(thrust::raw_pointer_cast(metadata.data()),
                                 host_metadata.data(),
                                 host_metadata.size() * sizeof(CubqlMetadata),
                                 cudaMemcpyHostToDevice, stream));
    }
  }

  void query_device(int2* output, int output_capacity) {
    const auto& query_boxes =
        input_.kind == QueryKind::EE ? boxes_a_ : boxes_b_;
    const auto& query_metadata =
        input_.kind == QueryKind::EE ? metadata_a_ : metadata_b_;
    cubql_query_kernel<<<(query_boxes.size() + 127) / 128, 128, 0, stream_>>>(
        thrust::raw_pointer_cast(query_boxes.data()),
        thrust::raw_pointer_cast(query_metadata.data()), query_boxes.size(),
        thrust::raw_pointer_cast(boxes_a_.data()),
        thrust::raw_pointer_cast(metadata_a_.data()), bvh_,
        input_.kind == QueryKind::EE, output, output_capacity,
        thrust::raw_pointer_cast(output_num_.data()));
    CHECK_CUDA(cudaGetLastError());
  }

  cudaStream_t stream_ = nullptr;
  QueryInput input_;
  cuBQL::bvh3f bvh_;
  std::vector<cuBQL::box3f> host_boxes_a_;
  std::vector<cuBQL::box3f> host_boxes_b_;
  std::vector<CubqlMetadata> host_metadata_a_;
  std::vector<CubqlMetadata> host_metadata_b_;
  thrust::device_vector<cuBQL::box3f> boxes_a_;
  thrust::device_vector<cuBQL::box3f> boxes_b_;
  thrust::device_vector<CubqlMetadata> metadata_a_;
  thrust::device_vector<CubqlMetadata> metadata_b_;
  thrust::device_vector<int2> output_device_;
  thrust::device_vector<int> output_num_;
  std::vector<int2> native_output_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<GpuAdapter> make_cubql_adapter() {
  return std::make_unique<CubqlAdapter>();
}

}  // namespace silk::broadphase_benchmark
