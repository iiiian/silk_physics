#pragma once

#include <Eigen/Core>
#include <vector>

#include "collision/cpu/bbox.hpp"
#include "collision/cpu/collision.hpp"
#include "ecs.hpp"

namespace silk {

/**
 * @brief Collision detection and resolution pipeline.
 */
class CpuCollisionPipeline {
 public:
  float ccd_tolerance = 1e-6f;
  int ccd_max_iter = 1024;
  int partial_ccd_max_iter = 32;

  // Minimum time-of-impact to prevent infinite solver loop.
  float min_toi = 0.05f;

  float collision_stiffness_base = 1e4f;
  float collision_stiffness_max = 1e4f;
  float collision_stiffness_growth = 16.0f;

  /**
   * @brief Detect collisions by running broad- and narrow-phase CCD.
   * @param registry ECS registry providing colliders to test and update.
   * @param scene_bbox Axis-aligned bounds enclosing the scene for error
   * metrics.
   * @param dt Simulation timestep size in seconds.
   * @return All collisions detected during the timestep.
   */
  std::vector<Collision> find_collision(Registry& registry,
                                        const Bbox& scene_bbox, float dt);

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
