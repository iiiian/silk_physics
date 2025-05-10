#include "collision_detector.hpp"

#include <embree4/rtcore.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cassert>

namespace eg = Eigen;

void CollisionDetector::rtc_bound_callback(
    const RTCBoundsFunctionArguments* args) {
  assert(args->geometryUserPtr);
  CollisionDetector* self =
      static_cast<CollisionDetector*>(args->geometryUserPtr);
  unsigned int f = args->primID;
  assert(self->pF_);
  assert((f < self->pF_->rows()));
  auto vidx = self->pF_->row(f);

  using Vec = eg::Matrix<float, 1, 3>;

  // the bounding box is simply the union bbox of the current triagnle and the
  // future triagnle
  Vec min = self->pV_->row(vidx(0));
  Vec max = self->pV_->row(vidx(0));
  min = min.cwiseMin(self->pV_->row(vidx(1)));
  max = max.cwiseMax(self->pV_->row(vidx(1)));
  min = min.cwiseMin(self->pV_->row(vidx(2)));
  max = max.cwiseMax(self->pV_->row(vidx(2)));
  min = min.cwiseMin(self->pV_next_->row(vidx(0)));
  max = max.cwiseMax(self->pV_next_->row(vidx(0)));
  min = min.cwiseMin(self->pV_next_->row(vidx(1)));
  max = max.cwiseMax(self->pV_next_->row(vidx(1)));
  min = min.cwiseMin(self->pV_next_->row(vidx(2)));
  max = max.cwiseMax(self->pV_next_->row(vidx(2)));

  RTCBounds& bbox = args->bounds_o[0];
  bbox.lower_x = min.x();
  bbox.lower_y = min.y();
  bbox.lower_z = min.z();
  bbox.upper_x = max.x();
  bbox.upper_y = max.y();
  bbox.upper_z = max.z();
}

CollisionDetector::CollisionDetector() {
  // device
  device_ = rtcNewDevice(nullptr);
  auto error_callback = [](void*, RTCError err, const char* message) {
    spdlog::error("Embree Error: {}", message);
  };
  rtcSetDeviceErrorFunction(device_, error_callback, nullptr);

  // scene
  scene_ = rtcNewScene(device_);
  rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_DYNAMIC);
  rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_MEDIUM);

  // moving triangle geometry
  moving_triangle_geom_ = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserData(moving_triangle_geom_, this);
  rtcSetGeometryBoundsFunction(moving_triangle_geom_,
                               CollisionDetector::rtc_bound_callback, nullptr);
  rtcAttachGeometry(scene_, moving_triangle_geom_);
  rtcReleaseGeometry(moving_triangle_geom_);
}

CollisionDetector::~CollisionDetector() {
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);
}

void CollisionDetector::init(unsigned int face_num) {
  rtcSetGeometryUserPrimitiveCount(moving_triangle_geom_, face_num);
}

void CollisionDetector::detect(const RMatrixX3f* pV, const RMatrixX3f* pV_next,
                               const RMatrixX3i* pF, RTCCollideFunc callback,
                               void* callback_data) {
  assert(pV);
  assert(pV_next);
  assert(pF);

  pV_ = pV;
  pV_next_ = pV_next;
  pF_ = pF;

  // update bvh
  rtcCommitGeometry(moving_triangle_geom_);
  rtcCommitScene(scene_);

  rtcCollide(scene_, scene_, callback, callback_data);
}
