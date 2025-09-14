/**
 * @file collision_pipeline.hpp
 * @brief Main collision detection and resolution pipeline for physics
 * simulation.
 *
 * This pipeline orchestrates multi-threaded collision detection between objects
 * and within objects (self-collision), using hierarchical broadphase and
 * continuous collision detection.
 */
#pragma once

#include <Eigen/Core>
#include <vector>

#include "collision.hpp"
#include "object_collider.hpp"

namespace silk {

/**
 * @brief Collision detection and resolution pipeline.
 */
class CollisionPipeline {
 public:
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  int partial_ccd_max_iter = 32;

  // Minimum time-of-impact to prevent infinite solver loop.
  float min_toi = 0.05f;

  float collision_stiffness_base = 1e4f;
  float collision_stiffness_max = 1e4f;
  float collision_stiffness_growth = 16.0f;

  std::vector<Collision> find_collision(
      std::vector<ObjectCollider>& object_colliders, const Bbox& scene_bbox,
      float dt);

  /**
   * @brief Partial CCD update.
   *
   * Currently this is not used anywhere.
   *
   * @param global_state_t0 Global state vector at start of timestep
   * @param global_state_t1 Global state vector after position update
   * @param collisions Collision list to update with new contact data
   */
  void update_collision(const Eigen::VectorXf& global_state_t0,
                        const Eigen::VectorXf& global_state_t1,
                        std::vector<Collision>& collisions) const;

 private:
  /** @brief Numerical error tolerance for edge-edge CCD queries. */
  Eigen::Array3f scene_ee_err_;
  /** @brief Numerical error tolerance for vertex-face CCD queries. */
  Eigen::Array3f scene_vf_err_;
};

}  // namespace silk
