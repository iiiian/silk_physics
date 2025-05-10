#pragma once

#include <embree4/rtcore.h>

#include <Eigen/Core>

#include "common_types.hpp"

// wrapper for embree bvh tree
class CollisionDetector {
  RTCDevice device_;
  RTCScene scene_;
  RTCGeometry moving_triangle_geom_;
  const RMatrixX3f* pV_ = nullptr;
  const RMatrixX3f* pV_next_ = nullptr;
  const RMatrixX3i* pF_ = nullptr;

  static void rtc_bound_callback(const RTCBoundsFunctionArguments* args);

 public:
  CollisionDetector();
  ~CollisionDetector();

  void init(unsigned int face_num);
  void detect(const RMatrixX3f* pV, const RMatrixX3f* pV_next,
              const RMatrixX3i* pF, RTCCollideFunc callback,
              void* callback_data);
};
