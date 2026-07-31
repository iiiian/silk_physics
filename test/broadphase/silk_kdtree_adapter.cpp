#include <utility>

#include "cpu_adapters.hpp"
#include "silk_cpu_adapter_common.hpp"

namespace silk::broadphase_benchmark {

namespace {

class SilkKdTreeAdapter : public CpuAdapter {
 public:
  using Tree = silk::cpu::KDTree<SilkCollider>;

  std::string name() const override { return "silk_kdtree"; }

  void prepare(const QueryInput& input) override {
    clear_output();
    std::vector<SilkCollider> a = make_silk_colliders(input.group_a);
    std::vector<SilkCollider> b = input.group_b.empty()
                                      ? std::vector<SilkCollider>{}
                                      : make_silk_colliders(input.group_b);
    if (!built_) {
      kind_ = input.kind;
      prepared_a_ = std::move(a);
      prepared_b_ = std::move(b);
      return;
    }
    tree_a_.get_colliders() = std::move(a);
    if (kind_ == QueryKind::VF) {
      tree_b_.get_colliders() = std::move(b);
    }
  }

  void build() override {
    silk::cpu::Bbox root;
    // Init tree at first build.
    if (!built_) {
      root = root_bbox(prepared_a_, prepared_b_);
      tree_a_.init(std::move(prepared_a_));
      if (kind_ == QueryKind::VF) {
        tree_b_.init(std::move(prepared_b_));
      }
      built_ = true;
    } else {
      root = kind_ == QueryKind::VF
                 ? root_bbox(tree_a_.get_colliders(), tree_b_.get_colliders())
                 : root_bbox(tree_a_.get_colliders());
    }
    tree_a_.update(root);
    // if kind == EE, this is a self collision query, no second tree.
    if (kind_ == QueryKind::VF) {
      tree_b_.update(root);
    }
  }

  void clear_output() override {
    cache_.clear();
    output_.clear();
  }

  void query() override {
    if (kind_ == QueryKind::EE) {
      tree_a_.test_self_collision(CanCollide{}, cache_);
    } else {
      silk::cpu::KDTree<SilkCollider>::test_tree_collision(tree_a_, tree_b_,
                                                           CanCollide{}, cache_);
    }
  }

  void materialize_output() override {
    output_.reserve(cache_.size());
    for (const auto& [a, b] : cache_) {
      output_.emplace_back(a->id, b->id);
    }
  }

  std::span<const Pair> output() const override { return output_; }

 private:
  QueryKind kind_ = QueryKind::EE;
  bool built_ = false;
  std::vector<SilkCollider> prepared_a_;
  std::vector<SilkCollider> prepared_b_;
  // For EE query, tree a stores edge bboxes and tree b is empty.
  // For VF query, tree a stores vert bboxes and tree b stores face bboxes.
  Tree tree_a_;
  Tree tree_b_;
  silk::cpu::CollisionCache<SilkCollider> cache_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<CpuAdapter> make_silk_kdtree_adapter() {
  return std::make_unique<SilkKdTreeAdapter>();
}

}  // namespace silk::broadphase_benchmark
