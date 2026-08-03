#pragma once

#include <Eigen/Core>
#include <vector>

#include "backend/cpu/collision/bbox.hpp"
#include "backend/cpu/collision/broadphase.hpp"
#include "backend/cpu/collision/collision.hpp"
#include "backend/cpu/collision/mesh_collider.hpp"
#include "backend/cpu/ecs.hpp"

namespace silk::cpu {

/// @brief Collision detection and resolution pipeline.
class CollisionPipeline {
 public:
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 4096;

  // TODO let user set cdd and dcd distance.
  // Bbox padding in broadphase is set to 5% edge length. Currently we scale
  // ccd/dcd distance based on that.

  /// CCD minimal separation distance = scale * minimum scene padding.
  float ccd_minimum_separation_scale = 0.2f;
  /// DCD activation distance = scale * minimum scene padding.
  float dcd_activation_distance_scale = 1.0f;

  float collision_stiffness_base = 1e4f;

  /// @brief Find the earliest CCD hit without constructing active contacts.
  /// @param registry ECS registry providing colliders to test and update.
  /// @param scene_bbox Axis-aligned bounds enclosing the scene for error
  /// metrics.
  /// @param time_start Absolute normalized time already consumed.
  /// @param max_time Maximum remaining normalized time to search.
  /// @return Earliest conservative TOI, or max_time if no CCD hit exists.
  float find_earliest_toi(Registry& registry, const Bbox& scene_bbox,
                          float time_start, float max_time);

  /// @brief Build the active contact set using DCD at one fixed state.
  /// @param frame_time Absolute normalized frame time in [0,1].
  std::vector<Collision> find_active_collisions(Registry& registry,
                                                float frame_time);

 private:
  CollisionCache<MeshCollider> mesh_ccache_;
};

}  // namespace silk::cpu
