#include <embree4/rtcore.h>
#include <tbb/enumerable_thread_specific.h>

#include <stdexcept>

#include "cpu_adapters.hpp"

namespace silk::broadphase_benchmark {

namespace {

using ThreadOutputs = tbb::enumerable_thread_specific<std::vector<Pair>>;

struct EmbreeQueryContext {
  const QueryInput* input = nullptr;
  ThreadOutputs* thread_outputs = nullptr;
};

struct EmbreeGeometryData {
  std::span<const Box> boxes;
};

void embree_error(void*, RTCError code, const char* message) {
  if (code != RTC_ERROR_NONE) {
    throw std::runtime_error(message == nullptr ? "Embree error" : message);
  }
}

void embree_bounds(const RTCBoundsFunctionArguments* arguments) {
  const auto* geometry =
      static_cast<const EmbreeGeometryData*>(arguments->geometryUserPtr);
  const Box& box = geometry->boxes[arguments->primID];
  arguments->bounds_o->lower_x = box.min.x();
  arguments->bounds_o->lower_y = box.min.y();
  arguments->bounds_o->lower_z = box.min.z();
  arguments->bounds_o->upper_x = box.max.x();
  arguments->bounds_o->upper_y = box.max.y();
  arguments->bounds_o->upper_z = box.max.z();
}

void embree_collide(void* user_ptr, RTCCollision* collisions,
                    unsigned int collision_num) {
  auto& context = *static_cast<EmbreeQueryContext*>(user_ptr);
  std::vector<Pair>& local = context.thread_outputs->local();
  for (unsigned int i = 0; i < collision_num; ++i) {
    Pair pair{static_cast<int>(collisions[i].primID0),
              static_cast<int>(collisions[i].primID1)};
    if (context.input->kind == QueryKind::EE && pair.first >= pair.second) {
      continue;
    }
    const Box& box_a = context.input->group_a[pair.first];
    const Box& box_b = context.input->kind == QueryKind::EE
                           ? context.input->group_a[pair.second]
                           : context.input->group_b[pair.second];
    if (!is_sharing_vertex(box_a, box_b) && is_boxes_overlaping(box_a, box_b)) {
      local.push_back(pair);
    }
  }
}

class EmbreeAdapter : public CpuAdapter {
 public:
  EmbreeAdapter() {
    device_ = rtcNewDevice(nullptr);
    if (device_ == nullptr) {
      throw std::runtime_error("Unable to create Embree device");
    }
    rtcSetDeviceErrorFunction(device_, embree_error, nullptr);
  }

  ~EmbreeAdapter() override {
    release_scenes();
    rtcReleaseDevice(device_);
  }

  std::string name() const override { return "embree"; }

  void prepare(const QueryInput& input) override {
    input_ = input;
    geometry_data_a_.boxes = input.group_a;
    geometry_data_b_.boxes = input.group_b;
    clear_output();
  }

  void build() override {
    if (scene_a_ == nullptr) {
      scene_a_ = build_scene(geometry_data_a_, geometry_a_);
      if (input_.kind == QueryKind::VF) {
        scene_b_ = build_scene(geometry_data_b_, geometry_b_);
      }
      return;
    }

    // The callback reads the current frame through geometry_data_*. Recommit
    // the geometry to refresh its primitive bounds and let Embree refit the
    // existing acceleration structure instead of rebuilding the scene.
    rtcCommitGeometry(geometry_a_);
    if (input_.kind == QueryKind::VF) {
      rtcCommitGeometry(geometry_b_);
    }
    rtcCommitScene(scene_a_);
    if (input_.kind == QueryKind::VF) {
      rtcCommitScene(scene_b_);
    }
  }

  void clear_output() override {
    output_.clear();
    for (std::vector<Pair>& local : thread_outputs_) {
      local.clear();
    }
  }

  void query() override {
    EmbreeQueryContext context{&input_, &thread_outputs_};
    // If kind == EE -> self collision test.
    // If kind == VF -> tree tree collision test.
    rtcCollide(scene_a_, input_.kind == QueryKind::EE ? scene_a_ : scene_b_,
               embree_collide, &context);
    // merge thread local results.
    size_t output_num = 0;
    for (const std::vector<Pair>& local : thread_outputs_) {
      output_num += local.size();
    }
    output_.reserve(output_num);
    for (const std::vector<Pair>& local : thread_outputs_) {
      output_.insert(output_.end(), local.begin(), local.end());
    }
  }

  std::span<const Pair> output() const override { return output_; }

 private:
  RTCScene build_scene(EmbreeGeometryData& geometry_data,
                       RTCGeometry& geometry_o) {
    RTCScene scene = rtcNewScene(device_);
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_LOW);
    geometry_o = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryBuildQuality(geometry_o, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(geometry_o, geometry_data.boxes.size());
    rtcSetGeometryUserData(geometry_o, &geometry_data);
    rtcSetGeometryBoundsFunction(geometry_o, embree_bounds, nullptr);
    rtcCommitGeometry(geometry_o);
    rtcAttachGeometry(scene, geometry_o);
    rtcCommitScene(scene);
    return scene;
  }

  void release_scenes() {
    if (scene_a_ != nullptr) {
      rtcReleaseScene(scene_a_);
      scene_a_ = nullptr;
    }
    if (scene_b_ != nullptr) {
      rtcReleaseScene(scene_b_);
      scene_b_ = nullptr;
    }
    if (geometry_a_ != nullptr) {
      rtcReleaseGeometry(geometry_a_);
      geometry_a_ = nullptr;
    }
    if (geometry_b_ != nullptr) {
      rtcReleaseGeometry(geometry_b_);
      geometry_b_ = nullptr;
    }
  }

  RTCDevice device_ = nullptr;
  RTCScene scene_a_ = nullptr;
  RTCScene scene_b_ = nullptr;
  RTCGeometry geometry_a_ = nullptr;
  RTCGeometry geometry_b_ = nullptr;
  EmbreeGeometryData geometry_data_a_;
  EmbreeGeometryData geometry_data_b_;
  QueryInput input_;
  ThreadOutputs thread_outputs_;
  std::vector<Pair> output_;
};

}  // namespace

std::unique_ptr<CpuAdapter> make_embree_adapter() {
  return std::make_unique<EmbreeAdapter>();
}

}  // namespace silk::broadphase_benchmark
