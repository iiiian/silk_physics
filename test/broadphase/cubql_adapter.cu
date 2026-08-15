#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "backend/cuda/collision/cubql_bvh.cuh"
#include "gpu_adapter_common.cuh"

namespace silk::broadphase_benchmark {
namespace {

struct GpuCollider {
  silk::cuda::Bbox bbox;
  int vertex_ids[3];
  int id;
};

silk::cuda::Vec3f make_silk_vec(const Eigen::Vector3f& value) {
  silk::cuda::Vec3f result;
  for (int i = 0; i < 3; ++i) {
    result(i) = value(i);
  }
  return result;
}

std::vector<GpuCollider> make_gpu_colliders(std::span<const Box> boxes) {
  std::vector<GpuCollider> result;
  result.reserve(boxes.size());
  for (const Box& box : boxes) {
    result.push_back({{make_silk_vec(box.min), make_silk_vec(box.max)},
                      {box.vertex_ids[0], box.vertex_ids[1], box.vertex_ids[2]},
                      box.id});
  }
  return result;
}

silk::cuda::Bbox make_root_bbox(std::span<const Box> a,
                                std::span<const Box> b) {
  Eigen::Vector3f min = a.front().min;
  Eigen::Vector3f max = a.front().max;
  for (const Box& box : a) {
    min = min.cwiseMin(box.min);
    max = max.cwiseMax(box.max);
  }
  for (const Box& box : b) {
    min = min.cwiseMin(box.min);
    max = max.cwiseMax(box.max);
  }
  return {make_silk_vec(min), make_silk_vec(max)};
}

__device__ bool gpu_share_vertex(const GpuCollider& a, const GpuCollider& b) {
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

class CubqlAdapter final : public GpuAdapter {
 private:
  using Tree = silk::cuda::CubqlBVH<GpuCollider>;
  using CollisionPair =
      silk::cuda::CubqlCollisionPair<GpuCollider, GpuCollider>;

 public:
  std::string name() const override { return "cubql"; }

  void prepare(const QueryInput& input) override {
    input_ = input;
    host_a_ = make_gpu_colliders(input.group_a);
    host_b_ = input.group_b.empty() ? std::vector<GpuCollider>{}
                                    : make_gpu_colliders(input.group_b);
    output_.clear();
    native_output_.clear();
  }

  void build() override {
    silk::cuda::Bbox root_bbox = make_root_bbox(input_.group_a, input_.group_b);
    if (!tree_.has_value()) {
      auto colliders_a = silk::cuda::vec_like_to_device<GpuCollider>(
          host_a_, context_.runtime());
      if (!host_b_.empty()) {
        colliders_b_ = silk::cuda::vec_like_to_device<GpuCollider>(
            host_b_, context_.runtime());
      }
      tree_.emplace(root_bbox, std::move(colliders_a), context_.runtime());
      collision_cache_ =
          silk::cuda::alloc<CollisionPair>(context_.runtime(), 1);
    } else {
      cu::copy_bytes(context_.stream, host_a_, tree_->get_colliders());
      if (input_.kind == QueryKind::VF) {
        cu::copy_bytes(context_.stream, host_b_, *colliders_b_);
      }
      tree_->update(root_bbox, context_.runtime());
    }
  }

  void clear_output() override {
    output_.clear();
    native_output_.clear();
  }

  void query() override {
    int count = 0;
    const auto filter = [] __device__(const GpuCollider& a,
                                      const GpuCollider& b) {
      return !gpu_share_vertex(a, b);
    };
    if (input_.kind == QueryKind::EE) {
      tree_->test_self_collision(filter, *collision_cache_, count,
                                 context_.runtime());
    } else {
      tree_->test_ext_collision<GpuCollider>(
          *colliders_b_, filter, *collision_cache_, count, context_.runtime());
    }

    if (count > 0) {
      native_output_ = silk::cuda::vec_like_to_host<CollisionPair>(
          ctd::span<const CollisionPair>(collision_cache_->data(), count),
          context_.runtime());
    }
  }

  void synchronize() override { context_.stream.sync(); }

  void materialize_output() override {
    const uintptr_t base_a =
        reinterpret_cast<uintptr_t>(tree_->view().colliders.data());
    const uintptr_t base_b =
        input_.kind == QueryKind::EE
            ? base_a
            : reinterpret_cast<uintptr_t>(colliders_b_->data());
    output_.reserve(native_output_.size());
    for (const CollisionPair& pair : native_output_) {
      int a = (reinterpret_cast<uintptr_t>(pair.first) - base_a) /
              sizeof(GpuCollider);
      int b = (reinterpret_cast<uintptr_t>(pair.second) - base_b) /
              sizeof(GpuCollider);
      output_.emplace_back(a, b);
    }
  }

  std::span<const Pair> output() const override { return output_; }

 private:
  GpuContext context_;
  QueryInput input_;
  std::vector<GpuCollider> host_a_;
  std::vector<GpuCollider> host_b_;
  std::optional<Tree> tree_;
  silk::cuda::Buf<GpuCollider> colliders_b_;
  silk::cuda::Buf<CollisionPair> collision_cache_;
  std::vector<CollisionPair> native_output_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<GpuAdapter> make_cubql_adapter() {
  return std::make_unique<CubqlAdapter>();
}

}  // namespace silk::broadphase_benchmark
